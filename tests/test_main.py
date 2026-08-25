import pytest

from ground_station.main import build_arg_parser


def test_host_is_required():
    parser = build_arg_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])


def test_port_defaults_to_9090_when_omitted():
    parser = build_arg_parser()
    args = parser.parse_args(["--host", "192.168.1.50"])

    assert args.host == "192.168.1.50"
    assert args.port == 9090


def test_port_accepts_and_coerces_int_when_given():
    parser = build_arg_parser()
    args = parser.parse_args(["--host", "192.168.1.50", "--port", "9999"])

    assert args.port == 9999
    assert isinstance(args.port, int)
