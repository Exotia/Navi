import argparse
import sys

from PySide6.QtWidgets import QApplication

from ground_station.ros_client import RosBridgeClient
from ground_station.ui.main_window import MainWindow


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Asterope ground station")
    parser.add_argument("--host", required=True, help="rosbridge websocket host, e.g. 192.168.1.50")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge websocket port (default 9090)")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    app = QApplication(sys.argv)
    client = RosBridgeClient(host=args.host, port=args.port)
    window = MainWindow(client)
    window.resize(1440, 900)
    window.show()

    client.connect()
    client.subscribe_cmd_vel()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
