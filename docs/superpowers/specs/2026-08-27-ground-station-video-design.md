# Ground Station Video — Design

Written 2026-08-27. Live video from the rover's ZED 2i to the ground station,
for manual driving. This is the first media path in the system; everything
before it was small JSON telemetry over rosbridge.

## Goal

The operator drives by what they see. That makes latency the hard constraint
(target well under 300 ms glass-to-glass) and image quality the thing that
gives way. The link in the field is a long-range, lossy WiFi connection, so
the stream must degrade into artifacts rather than freeze or stall.

Out of scope: depth and point cloud, the rear camera, recording, and any
still-image capture. One camera, one live view.

## Constraints found in the environment

Verified on 2026-08-27 against the real hardware:

- The ZED 2i enumerates as a plain UVC device at `/dev/video0` on the Orin
  and delivers **side-by-side stereo**. Default caps are `4416x1242@15`. The
  driving mode is the camera's VGA mode, `1344x376@30`, cropped to a single
  `672x376@30` eye — verified working on the hardware. Capture therefore
  needs neither the ZED SDK nor `zed_ros2_wrapper`, which keeps this path
  independent of the localization stack that owns them.
- Resolution is chosen for the link, not for the picture. `672x376` is about
  a seventh of the pixels of 720p and should hold the stream near
  600-800 kbit/s, which a long-range WiFi link can carry with margin. Driving
  needs obstacle and terrain recognition, not detail. `2560x720@30` also
  works and stays available as the upper bound a request may ask for.
- The Jetson hardware H.264 encoder is **not usable today**:
  `libgstnvvideo4linux2.so` is installed but `nvv4l2h264enc` fails to load.
  `x264enc` works and is the initial choice. Fixing the hardware encoder is a
  later optimization, not a blocker.
- The laptop has GStreamer but **no H.264 decoder** — `avdec_h264` is absent.
  `gstreamer1.0-libav` must be installed (needs sudo).
- The laptop has a GTX 1650, so hardware decode is available later if the CPU
  decode cost ever matters.

## Architecture

Two planes that deliberately share no transport:

- **Control plane** rides the existing rosbridge websocket link. Small,
  reliable, ordered — what TCP is good at.
- **Media plane** is H.264 over RTP/UDP, rover to laptop, never touching
  rosbridge. UDP is chosen for the field link: packet loss becomes brief
  artifacts instead of the stall-then-burst behavior TCP gives on a lossy
  connection.

Flow when the operator enables video:

1. The ground station determines its own address from the socket it already
   has open to the rover, then publishes a request carrying that address, a
   UDP port, and encoding parameters.
2. `video_sender` on the Orin validates the request and starts its pipeline.
3. Frames arrive at the laptop over UDP and are rendered in the video panel.
4. Disabling publishes a stop request, and the ground station also tears down
   its receiver locally — so an unreachable rover cannot leave a stream
   pointed at the operator.

The ground station sends its own address rather than the rover holding it in
configuration, because the rover is the server side of rosbridge and has no
other way to learn where the operator is. A hardcoded laptop address breaks
the moment the network changes.

### Why the request is a JSON string

`/video_request` and `/video_status` carry `std_msgs/String` holding JSON
rather than custom message types. A proper `.msg` would require an
`ament_cmake` package alongside the pure-Python `navi_teleop`, which is real
build complexity for two messages. The cost is accepted knowingly: the schema
lives in code on both ends instead of in an interface definition. If the
control surface grows beyond these two messages, revisit this.

## Components

### `video_sender` (Orin, `navi_teleop` package)

Subscribes `/video_request`, owns the send pipeline, publishes `/video_status`.

Request JSON: `enable`, `host`, `port`, `width`, `height`, `fps`,
`bitrate_kbps`.

Pipeline:

```
v4l2src device=/dev/video0
  ! video/x-raw,width=1344,height=376,framerate=30/1
  ! videocrop right=672
  ! videoconvert
  ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=<kbps> key-int-max=30
  ! rtph264pay config-interval=1 pt=96
  ! udpsink host=<host> port=<port>
```

`videocrop right=672` keeps the left eye of the side-by-side frame.
`key-int-max=30` bounds recovery after loss to about one second.
`config-interval=1` repeats SPS/PPS so a receiver that joins late, or after a
loss burst, can start decoding without the stream being restarted.

Defaults: `1344x376@30` capture, `800` kbit/s. The request may raise these,
but node parameters bound how far — maximum resolution `2560x720`, maximum
bitrate `4000` kbit/s, and the allowed capture device. An out-of-range or
malformed request is rejected into the `failed` state and never applied, so
the ground station cannot ask for more than the link can carry.

Status JSON: `state` (one of `stopped`, `starting`, `streaming`, `failed`)
and `detail` (free text, carrying the GStreamer error when relevant).

### `video_receiver` (laptop, `ground_station/video_receiver.py`)

Owns the receive pipeline. This is a plain Python module, not a ROS node —
the laptop has no ROS 2 installed, which is the reason rosbridge exists.

```
udpsrc port=<port> caps=application/x-rtp,media=video,encoding-name=H264,payload=96
  ! rtpjitterbuffer latency=100
  ! rtph264depay
  ! avdec_h264
  ! videoconvert
  ! <Qt sink>
```

It runs in the GUI process, owned by the panel. It also has a `__main__` so
it can be run standalone against a test stream with no rover and no GUI:
`python -m ground_station.video_receiver --port 5600`.

A separate process was considered and rejected: it would have to either
render into the panel's window handle or ship frames over a socket, and both
are more machinery than this feature justifies. Revisit only if decoder
crashes turn out to take the GUI down in practice.

### `VideoPanel` (`ground_station/ui/`)

Video surface, an on/off toggle, a status line reflecting `/video_status`,
and — separately — a local indicator for whether frames are actually
arriving. Keeping those two distinct is what tells the operator whether a
failure is upstream on the rover or on the wire.

### `RosBridgeClient`

Gains `publish_video_request()` and `subscribe_video_status()`, following the
existing `/manual_twist` pattern: roslibpy callbacks re-emitted as Qt signals
so they are marshaled onto the GUI thread.

## Error handling

- Loss degrades into artifacts, not stalls: UDP, a 100 ms jitter buffer, and
  a bounded keyframe interval so a corrupted stream self-heals within about a
  second.
- Toggling off stops the local receiver regardless of whether the rover
  answers.
- Rover-side pipeline errors surface on `/video_status` as `failed`, carrying
  the GStreamer message rather than a generic code.
- If no RTP packets arrive for two seconds while the rover reports
  `streaming`, the panel says so explicitly. That combination is the
  signature of a blocked UDP port, which the control path cannot detect on
  its own.
- Video failure never affects manual drive: separate topics, separate
  transport, separate node. A crashed `video_sender` leaves
  `manual_twist_listener` untouched.

## Testing

- **Unit, no hardware:** request validation and clamping, rejection of
  malformed JSON, status transitions.
- **Integration on the Orin:** `ros2 topic pub` a request and assert the node
  reaches `streaming`; assert an out-of-range request is refused.
- **Loopback, no rover:** `video_receiver` against `videotestsrc` pushed
  through the same encode chain, proving the receive path independently.
- **Manual end-to-end:** real camera to real panel, with the measured
  glass-to-glass latency recorded here once known.

## Prerequisites

- `sudo apt install gstreamer1.0-libav` on the laptop.
- The field network setup must pass UDP on the chosen port (default 5600).

## Deferred

- Jetson hardware encoder (`nvv4l2h264enc`), once its plugin load failure is
  diagnosed. Frees Orin CPU; no interface change.
- Adaptive bitrate. RTP/UDP has none: the operator picks a conservative
  bitrate for the link. If the field link proves too variable to tune by
  hand, the answer is WebRTC, which brings congestion control and
  retransmission at the cost of a signalling path and an embedded browser
  engine in the ground station.
- Rear camera, depth, and recording.
