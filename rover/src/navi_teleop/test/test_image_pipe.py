import pytest

from navi_teleop.image_pipe import (
    build_pipe_pipeline, bytes_per_pixel, gst_format_for,
    unsupported_frame_reason)


def test_the_pipeline_reads_whole_frames_from_stdin():
    argv = build_pipe_pipeline("192.168.178.101", 5600, 640, 360, 15, 800, "bgra8")
    assert argv[:2] == ["gst-launch-1.0", "-q"]
    assert "fdsrc" in argv and "fd=0" in argv
    # One blocksize = one frame, 640*360*4 bytes for bgra8.
    assert f"blocksize={640 * 360 * 4}" in argv


def test_the_pipeline_frames_the_byte_stream_with_rawvideoparse():
    # A capsfilter only asserts; rawvideoparse is what cuts a byte stream
    # into frames. Without it the decoder shows solid green.
    argv = build_pipe_pipeline("h", 5600, 640, 360, 15, 800, "bgra8")
    i = argv.index("rawvideoparse")
    assert argv[i + 1:i + 5] == ["width=640", "height=360", "format=bgra", "framerate=15/1"]


def test_the_pipeline_keeps_the_rover_encoder_settings():
    argv = build_pipe_pipeline("10.0.0.5", 5600, 640, 360, 15, 800, "bgra8")
    assert "x264enc" in argv
    assert "tune=zerolatency" in argv
    assert "bitrate=800" in argv
    assert "key-int-max=30" in argv
    assert "config-interval=1" in argv
    assert "host=10.0.0.5" in argv and "port=5600" in argv


@pytest.mark.parametrize("encoding,fmt,bpp", [
    ("bgra8", "bgra", 4), ("rgb8", "rgb", 3), ("bgr8", "bgr", 3)])
def test_encodings_map_to_gstreamer_formats(encoding, fmt, bpp):
    assert gst_format_for(encoding) == fmt
    assert bytes_per_pixel(encoding) == bpp


def test_an_unknown_encoding_is_refused_not_guessed():
    with pytest.raises(ValueError):
        gst_format_for("mono16")


def test_a_well_formed_frame_is_accepted():
    assert unsupported_frame_reason(640, 360, "bgra8", 640 * 360 * 4) is None


def test_any_geometry_is_accepted_because_the_stream_adopts_it():
    # The ground station asks for 1344x376 and the wrapper publishes
    # 640x360; the wrapper owns the camera, so the size it emits is the
    # only size there is. Refusing it only cost the operator their video.
    for width, height in [(1280, 720), (1344, 376), (640, 360), (16, 16)]:
        assert unsupported_frame_reason(
            width, height, "bgra8", width * height * 4) is None


def test_an_unsupported_encoding_is_refused_by_name():
    reason = unsupported_frame_reason(640, 360, "mono16", 640 * 360 * 2)
    assert "mono16" in reason


def test_a_short_frame_is_refused():
    # Geometry is adopted, but a frame that disagrees with its own header
    # is not: one torn frame desynchronises every frame after it.
    reason = unsupported_frame_reason(640, 360, "bgra8", 10)
    assert "bytes" in reason
