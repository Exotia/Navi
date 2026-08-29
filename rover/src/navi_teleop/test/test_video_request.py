import pytest

from navi_teleop.video_request import InvalidRequest, VideoRequest, parse_request

LIMITS = dict(max_width=2560, max_height=720, max_bitrate_kbps=4000,
              allowed_device="/dev/video0")


def test_parse_request_fills_defaults():
    request = parse_request('{"enable": true, "host": "192.168.178.101"}', **LIMITS)

    assert request == VideoRequest(enable=True, host="192.168.178.101", port=5600,
                                   width=1344, height=376, fps=30,
                                   bitrate_kbps=800, device="/dev/video0")


def test_parse_request_accepts_explicit_values():
    payload = ('{"enable": true, "host": "10.0.0.5", "port": 5601, "width": 2560, '
               '"height": 720, "fps": 15, "bitrate_kbps": 2500}')

    request = parse_request(payload, **LIMITS)

    assert (request.port, request.width, request.height) == (5601, 2560, 720)
    assert (request.fps, request.bitrate_kbps) == (15, 2500)


def test_parse_request_rejects_resolution_above_ceiling():
    payload = '{"enable": true, "host": "10.0.0.5", "width": 3840, "height": 1080}'

    with pytest.raises(InvalidRequest, match="width"):
        parse_request(payload, **LIMITS)


def test_parse_request_rejects_bitrate_above_ceiling():
    payload = '{"enable": true, "host": "10.0.0.5", "bitrate_kbps": 9000}'

    with pytest.raises(InvalidRequest, match="bitrate_kbps"):
        parse_request(payload, **LIMITS)


def test_parse_request_rejects_foreign_device():
    payload = '{"enable": true, "host": "10.0.0.5", "device": "/dev/video9"}'

    with pytest.raises(InvalidRequest, match="device"):
        parse_request(payload, **LIMITS)


def test_parse_request_rejects_malformed_json():
    with pytest.raises(InvalidRequest, match="JSON"):
        parse_request("{not json", **LIMITS)


def test_parse_request_requires_host_when_enabling():
    with pytest.raises(InvalidRequest, match="host"):
        parse_request('{"enable": true}', **LIMITS)


def test_parse_request_allows_disable_without_host():
    request = parse_request('{"enable": false}', **LIMITS)

    assert request.enable is False
    assert request.host == ""


def test_parse_request_rejects_port_outside_user_range():
    payload = '{"enable": true, "host": "10.0.0.5", "port": 80}'

    with pytest.raises(InvalidRequest, match="port"):
        parse_request(payload, **LIMITS)
