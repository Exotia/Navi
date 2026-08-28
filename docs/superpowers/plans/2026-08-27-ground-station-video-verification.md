# Ground Station Video — End-to-End Verification

Written 2026-08-28. Companion to `docs/superpowers/specs/2026-08-27-ground-station-video-design.md`
and to `docs/superpowers/plans/2026-08-25-ground-station-manual-verification.md`, whose shape this
follows.

## Status: nothing below has been run

The rover (`a_navi`, `192.168.178.33`) is off the network as of 2026-08-28 — `ping` shows 100%
loss and `ssh` gives `No route to host` — so no step in this document could be executed, and none
was. One install is still outstanding: `ros-humble-rosbridge-suite` on the Orin.
`gstreamer1.0-libav` on this laptop is now installed (confirmed 2026-08-28: `dpkg -s
gstreamer1.0-libav` reports installed, and `gst-inspect-1.0 avdec_h264` succeeds) - this document
is the procedure to run once the Orin is reachable and rosbridge is installed there, written out
fully so it can be followed without re-deriving anything. Every step below is marked pending. No
latency number, no panel text, no pass/fail is reported here as observed, because none was: the
decode leg being installable is not the same as the end-to-end path having been run.

What *is* verified, from the six completed tasks that precede this one:

- **Rover node paths** (`video_sender.py`, `video_request.py`): request, refuse, and teardown were
  live-verified against the real ZED 2i on the Orin before it went offline (Task 2). The capture
  chain, the crop that keeps the left eye, and the encoder settings are confirmed to run on that
  hardware.
- **Laptop side**: 89 automated tests cover `video_receiver.py`, `VideoPanel`, and the
  `RosBridgeClient` video methods against fakes — request shaping, status parsing, the
  starving/no-frames detection, and the panel's text for every state.

What is **not** verified by anything that has run so far:

- The H.264 decode leg on the laptop. `avdec_h264` has never been exercised here — the package
  that provides it (`gstreamer1.0-libav`) is now installed (see Prerequisites below), but
  `video_receiver.py`'s `gst-launch-1.0` pipeline has still not been started against a real RTP
  stream even once; installing the decoder is not the same as running it against a stream.
- The full path end to end: real camera → `x264enc` → UDP over the actual field link → laptop
  decode → panel. Nothing has connected the verified rover half to the verified laptop half.
- Glass-to-glass latency. The design's target ("well under 300 ms") is a goal stated in the spec,
  not a measurement.
- That `video_sender` can be started as part of the Orin's own boot/launch script rather than by
  hand — see the `start_navi.sh` section below, which is a proposed patch, not an applied one.

## Prerequisites

All four must be true before Step 1. Three remain unmet right now.

- [x] **Laptop**: H.264 decode support installed. Done as of 2026-08-28: `dpkg -s
  gstreamer1.0-libav` reports installed, and `gst-inspect-1.0 avdec_h264` succeeds on this
  machine. This only means `avdec_h264` is present to be started — it does not mean
  `video_receiver.py`'s pipeline has been run against a real RTP stream (see "What is not verified"
  above); that still requires the rover side below.
  ```
  sudo apt install gstreamer1.0-libav
  ```

- [ ] **Orin**: rosbridge installed.
  ```
  sudo apt install ros-humble-rosbridge-suite
  ```
  This is the control-plane transport `/video_request` and `/video_status` ride on; without it
  there is no websocket for the ground station to connect to at all, let alone one to carry video
  control messages.

- [ ] **Network**: the rover reachable at `192.168.178.33`. Confirm with `ping 192.168.178.33`
  before starting anything else in this document — if that fails, nothing past it will work either
  and the failure will look confusing rather than obvious.

- [ ] **Network**: UDP port 5600 (the default video port) open between the laptop and the rover.
  This is the media plane and does not go through rosbridge's TCP connection at all, so a
  successful rosbridge connection is not evidence this port is open. There is no separate
  reachability test for a UDP port that isn't listening yet; the first real test of it is Step 3
  below.

## Procedure

### Step 1: start the rover side

```bash
ssh star@a_navi 'bash -lc "
source /opt/ros/humble/setup.bash && source ~/navi/install/local_setup.bash
ros2 run navi_teleop video_sender
"'
```

This is by hand, in its own shell, in addition to whatever launches `rosbridge_server` and
`manual_twist_listener` (currently `~/navi/start_navi.sh` — see below for why `video_sender` isn't
part of that script yet).

- [ ] Pending.

### Step 2: start the ground station and connect

```bash
./start_ground_station.sh
```

Connect to `192.168.178.33:9090` (the default the script already points at), then press
**Start video** in the video panel.

- [ ] Pending.

### Step 3: confirm the happy path

Expected: the status line reads `STREAMING <laptop-ip>:5600` (the `RosBridgeClient` sends the
laptop's own address, discovered from the socket already open to the rover, as part of the
request — see `_on_stream_requested` in `ground_station/ui/main_window.py` — and `video_sender`
echoes host and port back in its `detail` field), and the camera image appears in the panel within
about two seconds.

- [ ] Pending. Record here what was actually seen, not what was expected, once run.

## Latency measurement

Point the rover's camera at a phone stopwatch running to hundredths of a second, take a screenshot
of the ground station panel showing both the live image and the stopwatch it is pointed at in the
same frame, and subtract the stopwatch reading from the moment of the screenshot.

**Measured glass-to-glass latency: _____ ms — NOT YET MEASURED.**

This number has no value until it comes from a real screenshot; do not fill it in from the spec's
"well under 300 ms" target or from any other pipeline's typical numbers. Replace this line with the
measured figure and a one-line note of the conditions (bitrate, resolution, distance from the
router) when this procedure is actually run.

## Failure paths to confirm

Each expected text below is the literal string `VideoPanel._refresh_status` produces
(`ground_station/ui/video_panel.py`), not a paraphrase — check panel text against these exactly.

- [ ] **Kill `video_sender` mid-stream** (Ctrl-C the Step 1 process while the panel reads
  `STREAMING ...`). Expected: after about two seconds with no RTP packets arriving,
  `NO FRAMES - rover streaming, nothing arriving (UDP blocked?)`. This is expected because the
  rover's last published `/video_status` still says `streaming` — nothing tells it otherwise once
  its process is dead — while the panel's own frame timer notices nothing has arrived for
  `no_frame_after_seconds` (2.0s). This is the exact blocked-UDP signature the two-indicator design
  exists to surface; a panel that only looked at rover state would sit on `STREAMING` forever after
  the process died.

- [ ] **Press Start video while disconnected** (stop the ground station's rosbridge connection, or
  start it pointed at a host with nothing listening, then click **Start video**). Expected:
  `FAILED - not connected to rosbridge`, produced immediately and locally by
  `_on_stream_requested` without ever reaching the rover — `self.ros_client.is_connected` is
  checked before any request is published.

- [ ] **Unplug the ZED and press Start video**. Expected: `FAILED - <gstreamer error text>` within
  a couple of seconds, once `video_sender`'s `gst-launch-1.0` subprocess fails to open
  `/dev/video0` and the node reports that failure on `/video_status`. Record the exact `<detail>`
  text seen, since this is the one failure path whose message text depends on GStreamer/V4L2
  output rather than on this project's own code.

## Driving is not disturbed by video

- [ ] With video streaming (Step 3 confirmed) and a gamepad connected, confirm the ground
  station's Drive card keeps reporting its expected nonzero Hz, the same check used in
  `docs/superpowers/plans/2026-08-25-ground-station-manual-verification.md`. `/manual_twist` and
  `/video_request`/`/video_status` are different topics carried over the same rosbridge
  connection, but the drive path and the video path are separate nodes on the rover
  (`manual_twist_listener` vs `video_sender`) and separate processes, so nothing about the video
  pipeline should touch the Hz reading.
- [ ] Kill `video_sender` (as in the failure-path check above) while still driving, and confirm the
  Drive card's Hz reading is unaffected — this is the check that a crashed video node cannot take
  the drive path down with it.

## The `start_navi.sh` change

This is a **proposed patch, not an applied one** — it has not been tested, because it can only be
tested once the Orin is reachable again, at which point it should be re-verified against the
script's actual current text before being applied for real.

Today, `~/navi/start_navi.sh` starts `rosbridge_server` and `manual_twist_listener` only — noted
in this project's progress log by `grep -c video_sender start_navi.sh` returning `0` — so Step 1
above has to start `video_sender` by hand in a third shell every time. That's a gap worth closing: the script
should bring up all three together, the same way `start_ground_station.sh` on the laptop already
backgrounds its optional mock rosbridge process and tears it down with a `trap`.

The exact current line numbers of `start_navi.sh` are not available to write this diff against —
that file lives only in the `~/navi` repo on the Orin, was never copied into this repo, and the
Orin has been unreachable since before this task started. What follows describes the change in
terms of the structure already named in the Task 7 brief (a `rosbridge_server` launch plus a
`trap`-based cleanup) and in terms of the same idiom this repo's own `start_ground_station.sh`
already uses for its optional background process. **Before applying, re-read the live file and
adjust variable names, existing flags, and line numbers to match it** — do not paste this in
blind.

Proposed shape of the change:

1. Add a `--no-video` flag, parsed alongside whatever flags the script already accepts, that skips
   starting `video_sender` — for the case where the operator wants ROS and driving but not the
   camera (e.g. no camera plugged in, or debugging the video path separately as in Step 1 above).

2. Start `video_sender` in the background, after `rosbridge_server` is up, capturing its PID:

   ```bash
   VIDEO_PID=""
   if [ "$NO_VIDEO" -ne 1 ]; then
       ros2 run navi_teleop video_sender &
       VIDEO_PID=$!
   fi
   ```

3. Extend the script's existing `trap`-based cleanup (the same mechanism that already stops
   `rosbridge_server` on exit) to also stop `video_sender`:

   ```bash
   cleanup() {
       # ... existing cleanup for rosbridge_server, manual_twist_listener ...
       if [ -n "$VIDEO_PID" ] && kill -0 "$VIDEO_PID" 2>/dev/null; then
           kill "$VIDEO_PID" 2>/dev/null || true
           wait "$VIDEO_PID" 2>/dev/null || true
       fi
   }
   trap cleanup EXIT INT TERM
   ```

   This mirrors `start_ground_station.sh`'s own `MOCK_PID`/`cleanup`/`trap cleanup EXIT INT TERM`
   pattern in this repo (`start_ground_station.sh`, laptop side) — background an optional process,
   remember its PID only if started, and fold its teardown into the same trap that already
   cleans up the other launched process, so a Ctrl-C or a normal exit cannot leave `video_sender`
   running orphaned with no `rosbridge_server` left to receive its stop request.

4. Once applied, Step 1 of this document's Procedure section becomes unnecessary — `video_sender`
   would start automatically as part of the normal rover launch — and Step 1 should be rewritten
   to just start `start_navi.sh` (with `--no-video` as the way to reproduce today's by-hand
   behavior for isolated debugging). Do not make that rewrite until the patch itself has been
   applied and confirmed against the real script.

- [ ] Pending: apply, verify `grep -c video_sender start_navi.sh` now returns a nonzero count, and
  confirm a normal Ctrl-C stops all three processes (`rosbridge_server`, `manual_twist_listener`,
  `video_sender`) with none left running.
