#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "navi_sim_ik/sim_ik_stepper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

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
  : Node("sim_ik"), stepper_(0.06)
  {
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/manual_twist", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        yaw_rate_ = msg->angular.z;
        last_twist_ = now();
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

    timer_ = create_wall_timer(60ms, [this] {tick();});
  }

private:
  void tick()
  {
    // A twist that stops arriving means the link to the rover is gone. Coast
    // to a stop rather than continuing to drive on a stale command: a
    // simulation that keeps moving after the rover stopped talking is worse
    // than one that freezes, because it looks alive.
    const bool stale = (now() - last_twist_) > rclcpp::Duration::from_seconds(1.0);
    if (stale) {
      if (!reported_stale_) {
        RCLCPP_WARN(get_logger(), "/manual_twist is stale - is the rover reachable?");
        reported_stale_ = true;
      }
      vx_ = vy_ = yaw_rate_ = 0.0;
    } else if (reported_stale_) {
      RCLCPP_INFO(get_logger(), "/manual_twist is flowing again");
      reported_stale_ = false;
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
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (int i = 0; i < 4; ++i) {
      const std::string corner = navi_sim_ik::WHEEL_CORNERS[i];
      msg.joint_names.push_back("steer_" + corner + "_joint");
      point.positions.push_back(stepper_.targets().steer[i]);

      // The rolling angle is integrated here rather than commanded as a
      // velocity: joint_pose_trajectory sets positions, and a wheel that
      // never rotates makes a moving rover look like it is sliding.
      wheel_angle_[i] += stepper_.targets().spin[i] * 0.06;
      msg.joint_names.push_back("wheel_" + corner + "_joint");
      point.positions.push_back(wheel_angle_[i]);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(0.06);
    msg.points.push_back(point);
    joints_pub_->publish(msg);
  }

  void publish_motion()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = (stepper_.pose().x - last_pose_.x) / 0.06;
    cmd.linear.y = (stepper_.pose().y - last_pose_.y) / 0.06;
    cmd.angular.z = (stepper_.pose().yaw - last_pose_.yaw) / 0.06;
    last_pose_ = stepper_.pose();
    cmd_vel_pub_->publish(cmd);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = stepper_.pose().x;
    odom.pose.pose.position.y = stepper_.pose().y;
    odom.pose.pose.orientation.z = std::sin(stepper_.pose().yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(stepper_.pose().yaw / 2.0);
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
  navi_sim_ik::Pose2D last_pose_{};
  rclcpp::Time last_twist_{0, 0, RCL_ROS_TIME};
  bool reported_stale_{true};

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
