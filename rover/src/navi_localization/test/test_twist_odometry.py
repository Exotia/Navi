"""Tests for navi_localization.twist_odometry: the pure TwistIntegrator and
the ROS wiring around it.

  bash -c 'source /opt/ros/humble/setup.bash &&
    PYTHONPATH=$PWD/rover/src/navi_localization:$PWD/rover/src/navi_autonomy \
    python3 -m pytest rover/src/navi_localization/test/test_twist_odometry.py -q'
"""

import math

import pytest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from navi_localization.twist_odometry import TwistIntegrator, TwistOdometry


class FakeClock:
    """A monotonic-shaped clock the test drives by hand, the same fake-time
    idiom glare_watch.py's own tests use via its clock=monotonic injection
    point."""

    def __init__(self, t=0.0):
        self._t = t

    def __call__(self):
        return self._t

    def advance(self, dt):
        self._t += dt
        return self._t


def test_driving_straight_for_five_seconds_at_0_2_ms_lands_within_a_millimetre_of_one_metre():
    integrator = TwistIntegrator()
    t = 0.0
    dt = 0.04  # 25 Hz
    integrator.command(0.2, 0.0, t)
    for _ in range(int(5.0 / dt)):
        t += dt
        integrator.command(0.2, 0.0, t)
        integrator.gyro(0.0, t)
    assert integrator.x == pytest.approx(1.0, abs=1e-3)
    assert integrator.y == pytest.approx(0.0, abs=1e-3)
    assert integrator.distance_travelled == pytest.approx(1.0, abs=1e-3)


def test_a_ninety_degree_gyro_turn_then_straight_driving_moves_in_the_turned_direction():
    integrator = TwistIntegrator()
    t = 0.0
    integrator.command(0.2, 0.0, t)
    integrator.gyro(0.0, t)  # primes gyro()'s own clock; no heading change yet

    # Turn 90 degrees over one 1 s tick, with a fresh command riding along
    # so the drive-command timeout is not what this test is exercising.
    t += 1.0
    integrator.command(0.2, 0.0, t)
    integrator.gyro(math.radians(90.0), t)
    assert integrator.heading == pytest.approx(math.pi / 2, abs=1e-6)

    x_after_turn, y_after_turn = integrator.x, integrator.y
    dt = 0.04
    for _ in range(50):  # 2.0 s of straight driving after the turn
        t += dt
        integrator.command(0.2, 0.0, t)
        integrator.gyro(0.0, t)
    dx = integrator.x - x_after_turn
    dy = integrator.y - y_after_turn
    # Heading pi/2 means the body-frame +x command now drives in +y.
    assert dx == pytest.approx(0.0, abs=1e-3)
    assert dy == pytest.approx(0.4, abs=1e-3)


def test_a_stale_command_integrates_as_standstill_even_though_it_was_never_zeroed():
    integrator = TwistIntegrator(command_timeout_s=0.5)
    integrator.command(0.2, 0.0, 0.0)
    integrator.gyro(0.0, 0.4)  # still fresh: 0.4 s old
    x_before_timeout = integrator.x
    assert x_before_timeout > 0.0

    integrator.gyro(0.0, 2.0)  # 2.0 s since the command: well past 0.5 s
    assert integrator.x == pytest.approx(x_before_timeout, abs=1e-9)
    assert integrator.distance_travelled == pytest.approx(x_before_timeout, abs=1e-9)


def test_distance_travelled_accumulates_along_a_curved_path():
    integrator = TwistIntegrator()
    t = 0.0
    integrator.command(0.2, 0.0, t)
    for _ in range(200):  # 8 s of a hard turn: a full arc, not a gentle bend
        t += 0.04
        integrator.command(0.2, 0.0, t)
        integrator.gyro(1.0, t)
    # A curved path covers more ground than its straight-line displacement.
    straight_line = math.hypot(integrator.x, integrator.y)
    assert integrator.distance_travelled > straight_line + 1e-3


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin_once(node, clock, dt, publish_rate_hz=20.0):
    period = 1.0 / publish_rate_hz
    for _ in range(int(round(dt / period))):
        clock.advance(period)
        node._on_timer()


def test_commanded_wz_is_ignored_while_gyro_is_fresh_and_used_once_the_imu_goes_silent():
    clock = FakeClock()
    node = TwistOdometry(clock=clock)
    try:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1.0  # a large commanded turn the gyro must override
        node._on_twist(twist)

        imu = Imu()
        imu.angular_velocity.z = 0.0
        node._on_imu(imu)
        _spin_once(node, clock, 0.2)
        assert node._integrator.heading == pytest.approx(0.0, abs=1e-6)
        assert not node._warned_imu_stale

        # Let the IMU go silent past imu_timeout_s (default 1.0 s).
        _spin_once(node, clock, 1.2)
        assert node._warned_imu_stale
        heading_after_first_outage_tick = node._integrator.heading
        assert heading_after_first_outage_tick != pytest.approx(0.0, abs=1e-6)

        # The warning must not re-fire every tick of the same outage.
        _spin_once(node, clock, 0.2)
        assert node._warned_imu_stale
    finally:
        node.destroy_node()


def test_the_imu_stale_warning_resets_once_fresh_imu_data_returns():
    clock = FakeClock()
    node = TwistOdometry(clock=clock)
    try:
        imu = Imu()
        imu.angular_velocity.z = 0.0
        node._on_imu(imu)
        _spin_once(node, clock, 1.5)
        assert node._warned_imu_stale

        node._on_imu(imu)
        assert not node._warned_imu_stale
    finally:
        node.destroy_node()


def test_a_command_older_than_the_timeout_is_treated_as_standstill_by_the_node_too():
    clock = FakeClock()
    node = TwistOdometry(clock=clock)
    try:
        twist = Twist()
        twist.linear.x = 0.2
        node._on_twist(twist)  # sent once, never repeated and never zeroed
        _spin_once(node, clock, 0.6)  # already past command_timeout_s (0.5)
        x_once_stale = node._integrator.x
        assert x_once_stale > 0.0

        _spin_once(node, clock, 0.5)  # more time passes, still no new twist
        assert node._integrator.x == pytest.approx(x_once_stale, abs=1e-6)
    finally:
        node.destroy_node()


def test_covariance_grows_with_distance_travelled():
    clock = FakeClock()
    node = TwistOdometry(clock=clock)
    published = []
    node._odom_pub.publish = lambda msg: published.append(msg)
    try:
        twist = Twist()
        twist.linear.x = 0.2
        node._on_twist(twist)
        _spin_once(node, clock, 0.2)
        first_cov = published[-1].pose.covariance[0]

        for _ in range(50):
            node._on_twist(twist)
            _spin_once(node, clock, 0.2)
        later_cov = published[-1].pose.covariance[0]

        assert later_cov > first_cov > 0.0
    finally:
        node.destroy_node()


def test_the_published_odometry_carries_the_right_frames():
    clock = FakeClock()
    node = TwistOdometry(clock=clock)
    published = []
    node._odom_pub.publish = lambda msg: published.append(msg)
    try:
        _spin_once(node, clock, 0.1)
        msg = published[-1]
        assert msg.header.frame_id == 'twist_odom'
        assert msg.child_frame_id == 'base_footprint'
    finally:
        node.destroy_node()


def test_the_node_never_creates_a_transform_broadcaster():
    clock = FakeClock()
    node = TwistOdometry(clock=clock)
    try:
        # This node publishes NO tf - the ZED wrapper owns that transform.
        # A TransformBroadcaster opens a publisher on /tf the instant it is
        # constructed, so its plain absence from the node's attributes is
        # the check: nothing here should be capable of broadcasting tf.
        assert not any(
            type(v).__name__ == 'TransformBroadcaster'
            for v in vars(node).values())
        publisher_topics = [
            pub.topic_name for pub in node.publishers
        ]
        assert '/tf' not in publisher_topics
    finally:
        node.destroy_node()
