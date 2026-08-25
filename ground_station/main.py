import argparse
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Asterope ground station")
    parser.add_argument("--host", required=True, help="rosbridge websocket host, e.g. 192.168.1.50")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge websocket port (default 9090)")
    return parser


def _connect_to_rosbridge(client: RosBridgeClient) -> None:
    """Attempt the (blocking, up-to-~10s) rosbridge connection. Called after
    the Qt event loop is already running, so a slow or failed connection
    doesn't block the window from painting. roslibpy raises RosTimeoutError
    (and can raise other connection errors) when it can't reach the server —
    caught here so a dead/unreachable rosbridge leaves the window up showing
    "ROSBRIDGE: DISCONNECTED" instead of crashing the process."""
    try:
        client.connect()
        client.subscribe_cmd_vel()
    except Exception as exc:
        print(f"ground_station: failed to connect to rosbridge: {exc}", file=sys.stderr)


def main() -> None:
    args = build_arg_parser().parse_args()

    app = QApplication(sys.argv)
    client = RosBridgeClient(host=args.host, port=args.port)
    window = MainWindow(client)
    window.resize(1440, 900)
    window.show()

    # Deferred to after app.exec() starts the event loop (via singleShot(0,
    # ...)) so the window has already painted before the connection attempt
    # can block this thread for up to ~10s.
    QTimer.singleShot(0, lambda: _connect_to_rosbridge(client))

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
