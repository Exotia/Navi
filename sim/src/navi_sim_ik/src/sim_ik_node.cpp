#include <array>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navi_sim_ik/sim_ik_stepper.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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

    stepper_.step(vx_, vy_, yaw_rate_);
    publish_joints();
    publish_motion();
    publish_debug(stale);
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
    odom.header.frame_id = "odom";
    odom.child_frame_id = kBaseFootprintFrameId;
    odom.pose.pose.position.x = stepper_.pose().x;
    odom.pose.pose.position.y = stepper_.pose().y;
    odom.pose.pose.orientation.z = std::sin(stepper_.pose().yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(stepper_.pose().yaw / 2.0);
    // See kDeadReckoningVariance: an all-zero covariance asserts certainty,
    // and this pose has none to assert. The 6x6 row-major diagonal is
    // (x, y, z, roll, pitch, yaw). The twist covariance is set the same
    // way even though odom.twist is left at zero, so a consumer cannot
    // read that unpopulated zero as a confidently measured standstill.
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 6 + i] = kDeadReckoningVariance;
      odom.twist.covariance[i * 6 + i] = kDeadReckoningVariance;
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

  // Three states, not a bool: "nothing has ever arrived" must not be
  // reported in the words used for "it came back".
  enum class TwistFlow { NeverArrived, Flowing, Stale };
  TwistFlow flow_{TwistFlow::NeverArrived};
  // Separate from flow_ because flow_ forgets: it passes through Stale on
  // the first tick, before anything has ever arrived. See tick().
  bool ever_flowed_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
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
