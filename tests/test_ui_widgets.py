from ground_station.models import DriveState, NodeStatus
from ground_station.ui.drive_card import DriveCard
from ground_station.ui.node_list_widget import NodeListWidget


def test_drive_card_shows_dash_before_any_data(qtbot):
    card = DriveCard()
    qtbot.addWidget(card)
    assert "--" in card.vx_label.text()


def test_drive_card_updates_from_state(qtbot):
    card = DriveCard()
    qtbot.addWidget(card)
    state = DriveState()
    state.ingest(0.42, -0.05, 0.10, now=10.0)

    card.update_from(state)

    assert "0.42" in card.vx_label.text()
    assert "-0.05" in card.vy_label.text()
    assert "0.10" in card.wz_label.text()


def test_drive_card_emits_details_requested_on_click(qtbot):
    card = DriveCard()
    qtbot.addWidget(card)

    with qtbot.waitSignal(card.details_requested, timeout=1000):
        card.details_link.mousePressEvent(None)


def test_node_list_widget_starts_empty(qtbot):
    widget = NodeListWidget()
    qtbot.addWidget(widget)
    assert widget.row_count() == 0


def test_node_list_widget_shows_a_row_per_node(qtbot):
    widget = NodeListWidget()
    qtbot.addWidget(widget)
    statuses = [
        NodeStatus(name="/cmd_vel_bridge", alive=True, last_seen=10.0),
        NodeStatus(name="/amcl", alive=False, last_seen=1.0),
    ]

    widget.update_from(statuses)

    assert widget.row_count() == 2
    assert "cmd_vel_bridge" in widget.row_text(0)
    assert "amcl" in widget.row_text(1)
