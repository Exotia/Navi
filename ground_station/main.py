import argparse
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Asterope ground station")
    parser.add_argument("--host", default=None,
                         help="rosbridge websocket host, e.g. 192.168.1.50 "
                              "(optional - can also be typed into the app and connected from there)")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge websocket port (default 9090)")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    app = QApplication(sys.argv)
    window = MainWindow(ros_client_factory=RosBridgeClient, initial_host=args.host, initial_port=args.port)
    window.resize(1440, 900)
    window.show()

    if args.host:
        # Deferred to after app.exec() starts the event loop (via
        # singleShot(0, ...)) so the window has already painted before the
        # connection attempt can block this thread for up to ~10s.
        QTimer.singleShot(0, lambda: window._connect_to(args.host, args.port))

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
