#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navi_sim_ik/sim_ik_stepper.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "navi_sim_ik/external_pose.hpp"

namespace
{
// The one place the simulation's tick rate is written down. Every other use
// (the stepper's own ts, the timer period, the wheel-roll integration, and
// the trajectory point's time_from_start) derives from this rather than
// repeating the literal, because a literal that drifted out of step with the
// others would desync the visual wheel rotation from the commanded spin rate
// with no error and no log line - the same silent-when-wrong failure mode
// the wheel mapping is logged to guard against.
constexpr double kTimestepSeconds = 0.06;

// CRITICAL, and invisible from this file alone: the tick runs on the
// simulation clock, and a ROS-time timer can only notice time passing when
// /clock arrives. So the /clock *period must divide kTimestepSeconds*, or
// the timer quantises to something slower than this constant says and every
// integration below - the pose on /sim_odom and the wheel roll angle - falls
// behind the body Gazebo is actually moving, with no error and no log line.
// The same silent-when-wrong failure mode this constant exists to prevent.
//
// The /clock period is not set here. It is libgazebo_ros_init's publish_rate
// parameter, passed on the gazebo command line in
// navi_sim_bringup/launch/sim.launch.py, and it DEFAULTS TO 10 Hz - one edit
// away from reintroducing the bug. Measured at that default: 0.06 does not
// divide 0.1, the tick ran at 13.13 Hz instead of 16.67 (0.79x), and
// /sim_odom reported 7.204 m of travel where `gz model -m asterope -p` had
// the model 8.954 m further along. The launch file now asks for 100 Hz, so
// the granularity is 0.01 s and divides 0.06 exactly.
//
// The symptom to recognise, if this is ever wrong again: /sim_odom and
// /sim_cmd_vel publish slower than 1/kTimestepSeconds while `gz stats`
// reports a real-time factor near 1. check_tick_rate() below warns about
// exactly this, so it should not be silent a second time.

// gazebo_ros_joint_pose_trajectory resolves this as the reference link
// before it will apply any position; left empty, it rejects every message
// ("needs a reference link [] as frame_id, aborting") and no joint ever
// moves. base_link would be the natural reference, but it does not survive
// as its own SDF link: base_footprint_joint is fixed, so Gazebo's URDF->SDF
// conversion lumps base_link's geometry into base_footprint and only
// base_footprint remains as a named link. Confirmed empirically: frame_id
// "base_link" still aborts with "needs a reference link [base_link]"
// because no such link exists in the spawned model. The same link also
// names the odometry message's child frame, since it is the frame whose
// pose that odometry describes.
constexpr char kBaseFootprintFrameId[] = "base_footprint";

// How long /manual_twist may go quiet before the command is zeroed. Measured
// against a steady (wall) clock - see the comment at the point of use.
constexpr double kTwistStaleAfterRealSeconds = 1.0;

// The pose on /sim_odom is dead reckoning: it is integrated from the twist
// the IK was *commanded*, with no localisation, no wheel encoders and no
// terrain. It drifts without bound, and the vendored controller's 0.48 deg
// of steady-state toe alone yaws it about 0.0102 rad/s while driving
// dead straight. An all-zero covariance is the REP-105 way of claiming
// perfect certainty, which would invite a future consumer (an EKF, or
// whatever localisation lands here) to fuse this as ground truth - exactly
// the mistake the panel's "DEAD RECKONING, NO LOCALISATION" marker exists
// to stop a human making. This is the same statement addressed to a machine
// reader: large, deliberately uninformative variances. Not a calibrated
// number - there is nothing to calibrate against until localisation exists.
constexpr double kDeadReckoningVariance = 1.0e6;

// gazebo_ros_state's set_entity_state, under the /gazebo namespace the world
// file gives it. This is how the model is moved in semi-autonomous mode;
// planar_move is not loaded there, because two writers of one model pose
// fight and the picture jitters between them.
constexpr char kSetEntityStateService[] = "/gazebo/set_entity_state";

/// Heading out of a quaternion. The pose on /localization/pose is planar in
/// everything this simulation shows, so one angle is the whole of it.
double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

/// Drives the simulated rover from the same /manual_twist the real one gets.
///
/// The twist is read straight off the rover's ROS graph over DDS rather than
/// forwarded by the ground station: the ground station has no ROS and would
/// have needed a second rosbridge purely to repeat a message that is already
/// on the network.
class SimIkNode : public rclcpp::Node
{
public:
  SimIkNode()
  : Node("sim_ik"), stepper_(kTimestepSeconds)
  {
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/manual_twist", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        yaw_rate_ = msg->angular.z;
        // Deliberately the STEADY clock, not this node's ROS clock: the
        // staleness guard below asks "is the link to the rover still
        // there?", which is a wall-clock question. See tick().
        last_twist_ = steady_clock_.now();
        twist_ever_arrived_ = true;
      });

    joints_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/set_joint_trajectory", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/sim_cmd_vel", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim_odom", 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>("/sim_ik_debug", 10);

    // Logged every run because it is unverified and silent when wrong.
    RCLCPP_INFO(
      get_logger(), "wheel index mapping (UNVERIFIED): 0=%s 1=%s 2=%s 3=%s",
      navi_sim_ik::WHEEL_CORNERS[0], navi_sim_ik::WHEEL_CORNERS[1],
      navi_sim_ik::WHEEL_CORNERS[2], navi_sim_ik::WHEEL_CORNERS[3]);

    // Deliberately the node's OWN clock, not a wall timer. With
    // use_sim_time this is the simulation clock, which is the clock every
    // consumer of this node's output already runs on: planar_move
    // integrates /sim_cmd_vel over sim time, joint_pose_trajectory applies
    // positions against the sim clock, and the chase camera's update_rate
    // is sim time too. create_wall_timer is the steady clock
    // unconditionally and use_sim_time does not touch it, so under a
    // real-time factor below 1 the node would tick 1/RTF times too often:
    // /sim_odom would report several times the distance Gazebo actually
    // moved the body, and the wheel roll angle - integrated here, applied
    // as a sim-time position - would spin visibly faster than the rover
    // travels. Ticking on this clock makes kTimestepSeconds mean the same
    // thing to this node as it means to Gazebo. If the simulation runs
    // slower than real time the operator sees slow motion, which is honest
    // and internally consistent, rather than a picture that disagrees with
    // its own odometry.
    //
    // Do NOT "tidy" this and the steady clock in tick() into one clock.
    // They answer different questions and each choice is a fixed bug.
    pose_topic_ = declare_parameter<std::string>("pose_topic", "");
    status_topic_ = declare_parameter<std::string>("status_topic", "/localization/status");
    model_name_ = declare_parameter<std::string>("model_name", "asterope");
    pose_z_offset_ = declare_parameter<double>("pose_z_offset", 0.05);
    const double max_pose_rate_hz = declare_parameter<double>("max_pose_rate_hz", 30.0);
    gate_ = navi_sim_ik::ExternalPoseGate(max_pose_rate_hz);

    // Empty by default, so Simulation mode is byte for byte what it was:
    // no subscriptions, no service client, no behaviour to regress.
    if (!pose_topic_.empty()) {
      pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        pose_topic_, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          // Stored, not applied. Applying here would put the pose in
          // before this tick's step() integrates over it, and /sim_odom
          // would report localisation plus one tick of dead reckoning.
          // See apply_external_pose().
          pending_odom_ = msg;
        });
      status_sub_ = create_subscription<std_msgs::msg::String>(
        status_topic_, 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          gate_.set_state(navi_sim_ik::localization_state(msg->data));
        });
      set_state_client_ =
        create_client<gazebo_msgs::srv::SetEntityState>(kSetEntityStateService);
      RCLCPP_INFO(
        get_logger(),
        "external pose mode: body pose from %s, gated on %s, model '%s' placed "
        "through %s at up to %.1f Hz (z offset %.3f m). planar_move must NOT be "
        "loaded in this mode - two writers of one model pose fight.",
        pose_topic_.c_str(), status_topic_.c_str(), model_name_.c_str(),
        kSetEntityStateService, max_pose_rate_hz, pose_z_offset_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "dead-reckoning mode: the pose is integrated from /manual_twist and "
        "drifts without bound. Set pose_topic to place the model from the "
        "rover's own localisation instead.");
    }

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(kTimestepSeconds),
      [this] {tick();});
  }

private:
  void tick()
  {
    // A twist that stops arriving means the link to the rover is gone. Coast
    // to a stop rather than continuing to drive on a stale command: a
    // simulation that keeps moving after the rover stopped talking is worse
    // than one that freezes, because it looks alive.
    //
    // The age is measured on a STEADY clock while everything this node
    // integrates runs on the node clock above. That asymmetry is
    // deliberate. This check is not part of the kinematics; it detects that
    // a physical link to another machine has gone away, which happens in
    // real seconds. At a real-time factor of ~0.25 a one-second *sim-time*
    // window is over four real seconds of continued driving on a command
    // the rover stopped sending, which is most of the guard's point gone.
    // Do NOT collapse the two clocks: one clock here reintroduces either
    // the odometry/physics mismatch (wall timer) or a guard that stretches
    // with the real-time factor (sim-time age).
    const bool stale =
      !twist_ever_arrived_ ||
      (steady_clock_.now() - last_twist_) >
      rclcpp::Duration::from_seconds(kTwistStaleAfterRealSeconds);
    if (stale) {
      if (flow_ != TwistFlow::Stale) {
        // "Never arrived" and "stopped arriving" are different facts and
        // are not reported in the same words: at startup, before /clock
        // even exists, the node used to announce that the topic was
        // "flowing again" when not a single message had ever landed.
        if (flow_ == TwistFlow::NeverArrived) {
          RCLCPP_WARN(
            get_logger(),
            "no /manual_twist has arrived yet - is the rover reachable?");
        } else {
          RCLCPP_WARN(get_logger(), "/manual_twist is stale - is the rover reachable?");
        }
        flow_ = TwistFlow::Stale;
      }
      vx_ = vy_ = yaw_rate_ = 0.0;
    } else if (flow_ != TwistFlow::Flowing) {
      // "again" only if it really has flowed before. flow_ alone cannot
      // answer that: the very first tick almost always runs before the
      // first twist arrives, so flow_ has already moved NeverArrived ->
      // Stale by the time the first message lands, and keying the wording
      // off it would report the rover's first ever command as a recovery.
      // Observed in the launch log before this line existed.
      if (ever_flowed_) {
        RCLCPP_INFO(get_logger(), "/manual_twist is flowing again");
      } else {
        RCLCPP_INFO(get_logger(), "/manual_twist is flowing");
      }
      flow_ = TwistFlow::Flowing;
      ever_flowed_ = true;
    }

    check_tick_rate();

    stepper_.step(vx_, vy_, yaw_rate_);
    apply_external_pose();
    publish_joints();
    publish_motion();
    publish_debug(stale);
  }

  /// Warns once if the tick is not actually running at kTimestepSeconds.
  ///
  /// Cheap, because it needs nothing this node does not already have: the
  /// node clock is what the timer fires against, so the mean interval
  /// between ticks IS the quantisation the timer suffered. Individual
  /// intervals are useless for this - a ROS-time timer catches up after a
  /// coarse /clock jump, so they alternate between ~0 and one clock period
  /// - which is why this measures the mean over a window instead of any
  /// single gap. The mean is also the quantity that matters: it is exactly
  /// the factor by which the integrated pose and wheel roll fall behind.
  void check_tick_rate()
  {
    if (tick_rate_reported_) {
      return;
    }
    const rclcpp::Time current = now();
    // Before /clock arrives now() is 0. A ROS-time timer should not have
    // fired at all in that case, but a zero reading would put the window's
    // start at the epoch, so wait for a real one.
    if (current.nanoseconds() == 0) {
      return;
    }
    if (first_tick_.nanoseconds() == 0) {
      first_tick_ = current;
      return;
    }
    ++tick_samples_;
    // Ticks counted over one elapsed ROS-time span, NOT the mean of
    // per-tick deltas. The two differ and only this one is right: after a
    // coarse /clock jump the timer catches up by firing several times
    // against the same clock value, so those deltas are zero. Averaging
    // deltas silently discards the catch-up ticks and returns the /clock
    // period instead of the tick rate - it reported 10.00 Hz where the
    // node was really ticking 13.13 times a second. Counting every tick
    // over the whole span cannot make that mistake.
    if (tick_samples_ >= kTickRateSamples) {
      const double mean =
        (current - first_tick_).seconds() / static_cast<double>(tick_samples_);
      const double error = std::abs(mean - kTimestepSeconds) / kTimestepSeconds;
      if (error > kTickRateTolerance) {
        RCLCPP_WARN(
          get_logger(),
          "tick is running at %.2f Hz, not the configured %.2f Hz "
          "(mean interval %.4f s vs %.4f s). /sim_odom and the wheel roll "
          "will under-report travel by about this factor, silently. Most "
          "likely /clock is too coarse to divide the timestep: check "
          "libgazebo_ros_init's publish_rate on the gazebo command line in "
          "sim.launch.py (it defaults to 10 Hz, which does not divide 0.06).",
          1.0 / mean, 1.0 / kTimestepSeconds, mean, kTimestepSeconds);
      } else {
        RCLCPP_INFO(
          get_logger(), "tick rate confirmed: %.2f Hz (configured %.2f Hz)",
          1.0 / mean, 1.0 / kTimestepSeconds);
      }
      tick_rate_reported_ = true;
    }
  }

  /// Replaces the integrated body pose with the localised one, and moves the
  /// Gazebo model to match.
  ///
  /// Called between step() and the publishes, on purpose. In the
  /// subscription callback the pose would land before this tick's step()
  /// integrated over it and /sim_odom would report the localised pose plus
  /// one tick of dead reckoning - a small, permanent, plausible-looking
  /// error, which is the worst kind. Here, /sim_odom is exactly what
  /// localisation said.
  ///
  /// The wheels and steering are untouched: they keep coming from
  /// /manual_twist through the IK, so the picture still shows a rover
  /// steering and rolling rather than sliding. Only the body pose is
  /// external.
  void apply_external_pose()
  {
    if (!pending_odom_) {
      return;
    }
    // Steady, not the simulation clock, for the same reason the twist
    // staleness guard uses it: the 30 Hz cap protects a service and a
    // physics thread, which are real-time resources and do not slow down
    // when the real-time factor does.
    const double now = steady_clock_.now().seconds();
    if (!gate_.accept(now)) {
      if (!gate_.ok() && (!holding_ || holding_state_ != gate_.state())) {
        RCLCPP_WARN(
          get_logger(),
          "localisation reports '%s', not OK - holding the model still and "
          "ignoring poses on %s until it recovers.",
          gate_.state().empty() ? "(nothing yet)" : gate_.state().c_str(),
          pose_topic_.c_str());
        holding_ = true;
        holding_state_ = gate_.state();
      }
      return;
    }
    if (holding_) {
      RCLCPP_INFO(
        get_logger(), "localisation is OK again - the model follows the pose");
      holding_ = false;
      holding_state_.clear();
    }

    const auto odom = pending_odom_;
    pending_odom_.reset();
    const navi_sim_ik::Pose2D pose{
      odom->pose.pose.position.x, odom->pose.pose.position.y,
      yaw_of(odom->pose.pose.orientation)};
    stepper_.set_pose(pose);
    applied_odom_ = odom;
    send_entity_state(pose, odom->pose.pose.position.z);
  }

  void send_entity_state(const navi_sim_ik::Pose2D & pose, double pose_z)
  {
    if (!set_state_client_->service_is_ready()) {
      // Warned once and then again on the next outage, rather than at the
      // pose rate: at 30 Hz this would be 30 identical lines a second, and
      // the launch log is where the reason has to be findable.
      if (!set_state_missing_) {
        RCLCPP_WARN(
          get_logger(),
          "%s is not available - is libgazebo_ros_state.so loaded in the world "
          "file? The joints will move and the body will not.",
          kSetEntityStateService);
        set_state_missing_ = true;
      }
      return;
    }
    if (set_state_missing_) {
      RCLCPP_INFO(get_logger(), "%s is available again", kSetEntityStateService);
      set_state_missing_ = false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = model_name_;
    request->state.reference_frame = "world";
    request->state.pose.position.x = pose.x;
    request->state.pose.position.y = pose.y;
    // /localization/pose is map -> base_footprint: its z is the height of
    // the ground under the rover in the map frame - the same frame the
    // mapper's terrain tiles are drawn in - plus the spawn offset
    // (spawn_entity.py -z 0.05), without which the model would sink into
    // flat ground by that much on the first pose.
    request->state.pose.position.z = navi_sim_ik::model_z(pose_z, pose_z_offset_);
    request->state.pose.orientation.z = std::sin(pose.yaw / 2.0);
    request->state.pose.orientation.w = std::cos(pose.yaw / 2.0);

    // Fire and forget. This node spins single-threaded, so waiting on the
    // future here would deadlock the very spin that delivers the response.
    // The callback exists only so a refusal is not silent.
    set_state_client_->async_send_request(
      request,
      [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture future) {
        if (!future.get()->success) {
          RCLCPP_WARN_ONCE(
            get_logger(),
            "%s refused to move '%s' - is that the entity's name in the world? "
            "It must match spawn_entity.py's -entity argument.",
            kSetEntityStateService, model_name_.c_str());
        }
      });
  }

  void publish_joints()
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = now();
    msg.header.frame_id = kBaseFootprintFrameId;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < 4; ++i) {
      const std::string corner = navi_sim_ik::WHEEL_CORNERS[i];
      msg.joint_names.push_back("steer_" + corner + "_joint");
      point.positions.push_back(stepper_.targets().steer[i]);

      // The rolling angle is integrated here rather than commanded as a
      // velocity: joint_pose_trajectory sets positions, and a wheel that
      // never rotates makes a moving rover look like it is sliding.
      wheel_angle_[i] += stepper_.targets().spin[i] * kTimestepSeconds;
      msg.joint_names.push_back("wheel_" + corner + "_joint");
      point.positions.push_back(wheel_angle_[i]);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(kTimestepSeconds);
    msg.points.push_back(point);
    joints_pub_->publish(msg);
  }

  void publish_motion()
  {
    // Published straight from the model's own body-frame achieved velocity,
    // not recovered by differencing the world-frame pose: the pose is built
    // by rotating this same velocity into the world frame first, so
    // differencing it back out would only recover the world-frame value.
    // gazebo_ros_planar_move (the consumer) expects cmd_vel in the robot's
    // own frame and applies its own heading - handing it a world-frame
    // velocity would have it rotated a second time.
    const auto & achieved = stepper_.achieved_velocity();
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = achieved.vx;
    cmd.linear.y = achieved.vy;
    cmd.angular.z = achieved.yaw_rate;
    cmd_vel_pub_->publish(cmd);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    // The frame this pose is expressed in changes with where it comes from.
    // Publishing a localised, map-frame pose under frame_id "odom" would
    // quietly claim it is a drifting local estimate.
    odom.header.frame_id = applied_odom_ ? "map" : "odom";
    odom.child_frame_id = kBaseFootprintFrameId;
    odom.pose.pose.position.x = stepper_.pose().x;
    odom.pose.pose.position.y = stepper_.pose().y;
    odom.pose.pose.orientation.z = std::sin(stepper_.pose().yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(stepper_.pose().yaw / 2.0);
    // See kDeadReckoningVariance: an all-zero covariance asserts certainty,
    // and a dead-reckoned pose has none to assert. The 6x6 row-major
    // diagonal is (x, y, z, roll, pitch, yaw). The twist covariance is set
    // that way unconditionally, because odom.twist is left at zero either
    // way and a consumer must not read that unpopulated zero as a
    // confidently measured standstill.
    for (int i = 0; i < 6; ++i) {
      odom.twist.covariance[i * 6 + i] = kDeadReckoningVariance;
    }
    if (applied_odom_) {
      // The pose is localisation's and so is its uncertainty. Stamping
      // kDeadReckoningVariance onto a measured pose is the same kind of lie
      // as an all-zero covariance on a dead-reckoned one, pointing the
      // other way.
      odom.pose.covariance = applied_odom_->pose.covariance;
    } else {
      for (int i = 0; i < 6; ++i) {
        odom.pose.covariance[i * 6 + i] = kDeadReckoningVariance;
      }
    }
    odom_pub_->publish(odom);
  }

  void publish_debug(bool stale)
  {
    std::ostringstream json;
    json << "{\"indirect_mode\":" << (stepper_.indirect_mode() ? "true" : "false")
         << ",\"feasible_icr\":[" << stepper_.feasible_icr()[0] << ","
         << stepper_.feasible_icr()[1] << "]"
         << ",\"pose\":[" << stepper_.pose().x << "," << stepper_.pose().y << ","
         << stepper_.pose().yaw << "]"
         << ",\"twist_stale\":" << (stale ? "true" : "false") << "}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    debug_pub_->publish(msg);
  }

  navi_sim_ik::SimIkStepper stepper_;
  double vx_{0.0}, vy_{0.0}, yaw_rate_{0.0};
  std::array<double, 4> wheel_angle_{{0.0, 0.0, 0.0, 0.0}};
  // Steady, so the staleness window is real seconds regardless of the
  // simulation's real-time factor. last_twist_ carries the same clock type;
  // rclcpp throws on subtracting times from different clocks, which is the
  // compiler-adjacent guard against the two being mixed up later.
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_twist_{0, 0, RCL_STEADY_TIME};
  bool twist_ever_arrived_{false};

  // External pose (semi-autonomous mode). All of it is inert while
  // pose_topic_ is empty: no subscriptions are created and pending_odom_ is
  // never set, so tick() costs one null check more than it did.
  std::string pose_topic_;
  std::string status_topic_;
  std::string model_name_;
  double pose_z_offset_{0.05};
  navi_sim_ik::ExternalPoseGate gate_{30.0};
  nav_msgs::msg::Odometry::SharedPtr pending_odom_;
  nav_msgs::msg::Odometry::SharedPtr applied_odom_;
  // Two fields rather than one: holding_state_ can legitimately be "" (no
  // status has arrived at all), so an empty string cannot also mean "not
  // holding" without swallowing the first and most important warning.
  bool holding_{false};
  std::string holding_state_;
  bool set_state_missing_{false};

  // Three states, not a bool: "nothing has ever arrived" must not be
  // reported in the words used for "it came back".
  enum class TwistFlow { NeverArrived, Flowing, Stale };
  TwistFlow flow_{TwistFlow::NeverArrived};
  // Separate from flow_ because flow_ forgets: it passes through Stale on
  // the first tick, before anything has ever arrived. See tick().
  bool ever_flowed_{false};

  // check_tick_rate() state. The window is a few seconds of ticks: long
  // enough that the catch-up jitter after each /clock jump averages out,
  // short enough that the answer arrives while someone is still watching
  // the launch output.
  static constexpr int kTickRateSamples = 100;
  static constexpr double kTickRateTolerance = 0.05;
  rclcpp::Time first_tick_{0, 0, RCL_ROS_TIME};
  int tick_samples_{0};
  bool tick_rate_reported_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_state_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimIkNode>());
  rclcpp::shutdown();
  return 0;
}
