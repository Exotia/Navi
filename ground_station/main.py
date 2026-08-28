import argparse
import signal
import sys

from PySide6.QtCore import QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication

from ground_station import theme
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
    app.setFont(QFont(theme.FONT_FAMILY))
    window = MainWindow(ros_client_factory=RosBridgeClient, initial_host=args.host, initial_port=args.port)
    window.resize(1440, 900)
    window.show()

    # app.quit() (used by the SIGINT handler below) exits the event loop
    # directly and does not send closeEvent to open windows, so MainWindow's
    # own closeEvent alone would miss the Ctrl+C path. aboutToQuit fires on
    # every quit path (window close included, redundantly and harmlessly),
    # so it's the one place that reliably stops gst-launch-1.0 before this
    # process exits - otherwise it would be orphaned with udp/video_port
    # still bound (udpsrc's default reuse=true), ready to swallow the next
    # session's stream.
    app.aboutToQuit.connect(
        lambda: window.dashboard_page.video_panel.stop_receiver(keep_failed_reason=True))

    # GamepadReader (constructed inside MainWindow above) already restores
    # Python's default SIGINT handling, undoing pygame/SDL's hijack of it -
    # but a plain KeyboardInterrupt raised inside a QTimer callback is just
    # logged and ignored by Qt, not enough to actually stop app.exec(). This
    # explicit handler is what makes Ctrl+C actually quit the app.
    signal.signal(signal.SIGINT, lambda *args: app.quit())

    if args.host:
        # Deferred to after app.exec() starts the event loop (via
        # singleShot(0, ...)) so the window has already painted before the
        # connection attempt can block this thread for up to ~10s.
        QTimer.singleShot(0, lambda: window._connect_to(args.host, args.port))

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
