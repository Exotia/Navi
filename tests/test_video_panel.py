from ground_station.ui.video_panel import VideoPanel


class FakeReceiver:
    def __init__(self, frame=None):
        self.width = 4
        self.height = 2
        self.started = False
        self.stopped = False
        self.is_running = True
        self._frame = frame

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def read_frame(self):
        return self._frame


def test_panel_starts_idle(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)

    assert "OFF" in panel.status_label.text().upper()


def test_toggle_emits_stream_requested_true_then_false(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    requests = []
    panel.stream_requested.connect(requests.append)

    panel.toggle_button.click()
    panel.toggle_button.click()

    assert requests == [True, False]


def test_set_streaming_starts_and_stops_the_receiver(qtbot):
    receiver = FakeReceiver()
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)

    panel.set_streaming(True)
    assert receiver.started

    panel.set_streaming(False)
    assert receiver.stopped


def test_apply_status_shows_the_rover_reported_state(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)

    panel.apply_status({"state": "failed", "detail": "camera busy"})

    assert "FAILED" in panel.status_label.text().upper()
    assert "camera busy" in panel.status_label.text()


def test_panel_reports_no_frames_while_rover_claims_streaming(qtbot):
    # The rover saying 'streaming' while nothing arrives is exactly the
    # blocked-UDP-port case, and it must read differently from 'off'.
    panel = VideoPanel(receiver=FakeReceiver(frame=None), no_frame_after_seconds=0.0)
    qtbot.addWidget(panel)
    panel.apply_status({"state": "streaming", "detail": "10.0.0.5:5600"})

    panel.set_streaming(True)
    panel._poll_frame(now=100.0)
    panel._poll_frame(now=101.0)

    assert "NO FRAMES" in panel.status_label.text().upper()


def test_panel_clears_no_frames_once_a_frame_arrives(qtbot):
    receiver = FakeReceiver(frame=bytes(4 * 2 * 3))
    panel = VideoPanel(receiver=receiver, no_frame_after_seconds=0.0)
    qtbot.addWidget(panel)
    panel.apply_status({"state": "streaming", "detail": "10.0.0.5:5600"})
    panel.set_streaming(True)

    panel._poll_frame(now=100.0)

    assert "NO FRAMES" not in panel.status_label.text().upper()
    assert panel.image_label.pixmap() is not None
