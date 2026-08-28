from navi_sim_video.sender import build_send_pipeline


def test_the_pipeline_reads_raw_frames_from_stdin():
    # Frames arrive as ROS messages, so gst cannot fetch them itself - the
    # node writes them in. Same shape as the rover's sender, which pipes
    # rather than linking GStreamer into the process.
    argv = build_send_pipeline("127.0.0.1", 5601, 672, 376, 30, 800)
    assert argv[0] == "gst-launch-1.0"
    assert "fdsrc" in argv
    assert any("width=672" in part and "height=376" in part for part in argv)


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
