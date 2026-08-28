from navi_sim_video.sender import build_send_pipeline


def test_the_pipeline_reads_raw_frames_from_stdin():
    # Frames arrive as ROS messages, so gst cannot fetch them itself - the
    # node writes them in. Same shape as the rover's sender, which pipes
    # rather than linking GStreamer into the process.
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert argv[0] == "gst-launch-1.0"
    assert "fdsrc" in argv
    # Geometry now lives on rawvideoparse as separate tokens rather than in
    # one caps string; see test_the_pipeline_frames_the_byte_stream for why
    # the caps string was not enough.
    assert "width=672" in argv
    assert "height=376" in argv


def test_the_pipeline_encodes_for_low_latency():
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert "x264enc" in argv
    assert any("tune=zerolatency" in part for part in argv)
    # Repeats SPS/PPS so a receiver joining late can start decoding without
    # the stream being restarted.
    assert any("config-interval=1" in part for part in argv)


def test_the_pipeline_targets_the_simulation_port():
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert "port=5601" in argv
    assert "host=127.0.0.1" in argv


def test_the_bitrate_is_carried_through():
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 1500)
    assert any("bitrate=1500" in part for part in argv)


def test_the_pipeline_frames_the_byte_stream():
    """The regression test for the green screen.

    fdsrc reads a pipe, not a camera: it emits fixed-size chunks — 4096
    bytes by default — with no notion of where one frame ends. A capsfilter
    after it does not chunk anything; it only *asserts* that each buffer
    already is a 672x376 RGB frame, which at 758,016 bytes it is not. The
    encoder then compresses malformed buffers and the decoder, given nothing
    it can make sense of, emits its zeroed output buffer: solid green.

    Measured on a loopback with no simulation involved, sending a solid
    200/40/40 image: the caps-only pipeline decoded to R=0 G=131 B=0, and
    the same pipeline with rawvideoparse recovered R=197 G=36 B=38.

    rawvideoparse is what turns a byte stream into frames. blocksize makes
    fdsrc hand over whole frames at a time rather than 186 fragments each.
    """
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)

    assert "rawvideoparse" in argv, (
        "no element frames the byte stream - the decoder will emit green")
    assert f"blocksize={672 * 376 * 3}" in argv

    parse_at = argv.index("rawvideoparse")
    parse_args = argv[parse_at:parse_at + 5]
    assert "width=672" in parse_args
    assert "height=376" in parse_args
    assert "format=rgb" in parse_args
    assert "framerate=30/1" in parse_args


def test_the_frame_geometry_follows_its_arguments():
    # The green screen was invisible to the old tests because they only ever
    # checked the literals they passed in. Vary them.
    argv = build_send_pipeline("127.0.0.1", 5601, 320, 240, 15, 800)

    assert f"blocksize={320 * 240 * 3}" in argv
    parse_at = argv.index("rawvideoparse")
    parse_args = argv[parse_at:parse_at + 5]
    assert "width=320" in parse_args
    assert "height=240" in parse_args
    assert "framerate=15/1" in parse_args
