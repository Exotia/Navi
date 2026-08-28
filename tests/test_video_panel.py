from ground_station.ui.video_panel import VideoPanel


class FakeReceiver:
    def __init__(self, frame=None):
        self.width = 4
        self.height = 2
        self.started = False
        self.stopped = False
        self.is_running = True
        # A constructor-supplied frame is delivered once, like a real
        # VideoReceiver's read_frame() only returns a *new* frame once one
        # has actually accumulated - it never keeps re-returning the same
        # frame forever. That matters once _poll_frame drains in a loop
        # until read_frame() returns None (Important 3): an infinite supply
        # here would make that loop spin forever.
        self._queue = [frame] if frame is not None else []

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def queue_frames(self, *frames):
        """Lets a test hand back several frames across successive
        read_frame() calls within the same tick, in order - simulating
        several RTP frames having arrived since the last poll."""
        self._queue.extend(frames)

    def read_frame(self):
        if self._queue:
            return self._queue.pop(0)
        return None


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


def test_two_clicks_after_a_rejected_enable_both_emit_true(qtbot):
    # Important 2: click 1 emits True; a rejection (e.g. "not connected to
    # rosbridge") arrives via apply_status before set_streaming is ever
    # called, so _toggle_requested must not stay stuck at True - otherwise
    # click 2 (the operator's retry) would emit False, silently publishing
    # a stop on a flaky link.
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    requests = []
    panel.stream_requested.connect(requests.append)

    panel.toggle_button.click()
    panel.apply_status({"state": "failed", "detail": "not connected to rosbridge"})
    panel.toggle_button.click()

    assert requests == [True, True]


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


def test_poll_paints_only_the_newest_of_several_buffered_frames(qtbot):
    # Important 3: one read_frame() per 33 ms tick against a 30 fps sender
    # has ~0.3 frames/s of slack - any GUI pause backs up frames behind the
    # pipe. A single tick must drain to the newest and paint only that one,
    # not the oldest, so added latency drains instead of persisting.
    receiver = FakeReceiver()
    old_frame = bytes([10]) * (4 * 2 * 3)
    new_frame = bytes([200]) * (4 * 2 * 3)
    receiver.queue_frames(old_frame, new_frame)
    panel = VideoPanel(receiver=receiver)
    qtbot.addWidget(panel)
    panel.set_streaming(True)

    panel._poll_frame(now=100.0)

    assert receiver._queue == []
    painted = panel.image_label.pixmap().toImage().pixelColor(0, 0)
    assert painted.red() == 200


def test_panel_reports_a_dead_local_receiver_distinctly_from_udp_blocked(qtbot):
    # Important 1: a dead local pipeline (caps mismatch, missing decoder,
    # udpsrc bind failure, decoder crash) must not read as "UDP blocked?" -
    # that sends the operator to the firewall when the fault is local.
    receiver = FakeReceiver(frame=None)
    receiver.is_running = False
    panel = VideoPanel(receiver=receiver, no_frame_after_seconds=0.0)
    qtbot.addWidget(panel)
    panel.apply_status({"state": "streaming", "detail": "10.0.0.5:5600"})
    panel.set_streaming(True)

    panel._poll_frame(now=100.0)

    text = panel.status_label.text().upper()
    assert "UDP BLOCKED" not in text
    assert "NO FRAMES" in text
    assert "RECEIVER" in text


def test_starvation_warning_also_fires_while_rover_is_still_starting(qtbot):
    # Cheap item: previously this only fired for rover_state == "streaming",
    # so a rover stuck at "starting" showed STARTING forever with no local
    # corroboration.
    panel = VideoPanel(receiver=FakeReceiver(frame=None), no_frame_after_seconds=0.0)
    qtbot.addWidget(panel)
    panel.apply_status({"state": "starting", "detail": ""})
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


def test_stop_receiver_resets_to_stopped_like_an_operator_stop(qtbot):
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    panel.apply_status({"state": "failed", "detail": "camera busy"})

    panel.stop_receiver()

    assert panel.receiver.stopped
    assert "camera busy" not in panel.status_label.text()
    assert "OFF" in panel.status_label.text().upper()


def test_stop_receiver_keep_failed_reason_preserves_a_reported_failure(qtbot):
    # Critical 2's caveat: on disconnect/shutdown, a previously reported
    # 'failed' reason must survive - it explains what just happened and
    # set_streaming(False)'s ordinary reset-to-'stopped' would erase it.
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)
    panel.apply_status({"state": "failed", "detail": "camera busy"})

    panel.stop_receiver(keep_failed_reason=True)

    assert panel.receiver.stopped
    text = panel.status_label.text()
    assert "camera busy" in text
    assert "FAILED" in text.upper()


class RaisingReceiver(FakeReceiver):
    def start(self):
        raise RuntimeError("gst-launch-1.0: command not found")


def test_set_streaming_true_reports_failure_when_receiver_start_raises(qtbot):
    panel = VideoPanel(receiver=RaisingReceiver())
    qtbot.addWidget(panel)

    panel.set_streaming(True)

    assert panel._streaming is False
    assert panel.toggle_button.text() == "Start video"
    text = panel.status_label.text().upper()
    assert "FAILED" in text


def test_apply_status_streaming_before_local_start_is_not_plain_success(qtbot):
    # The rover's word alone is not enough: until this panel is actually
    # polling for frames, "streaming" must not render as healthy success,
    # since there is no local evidence yet to corroborate it.
    panel = VideoPanel(receiver=FakeReceiver())
    qtbot.addWidget(panel)

    panel.apply_status({"state": "streaming", "detail": "10.0.0.5:5600"})

    text = panel.status_label.text().upper()
    assert "NOT RECEIVING" in text
    assert not text.startswith("STREAMING ")
