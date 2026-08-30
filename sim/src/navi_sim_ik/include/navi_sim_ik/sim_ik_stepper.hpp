#ifndef NAVI_SIM_IK__SIM_IK_STEPPER_HPP_
#define NAVI_SIM_IK__SIM_IK_STEPPER_HPP_

#include <array>

#include "kinematics.h"
#include "navi_sim_ik/asterope_params.hpp"

namespace navi_sim_ik
{

/// Which physical corner each index of the model's beta/omega arrays refers to.
///
/// UNVERIFIED. The model indexes wheels 1-4, matching Steer Module 1-4 and
/// Drive Module 1-4 on the I2C bus, but nothing in this project records which
/// corner each module is bolted to - that is wiring knowledge. Getting it
/// wrong steers the wrong corners while looking entirely plausible, which is
/// why it is one named constant, meant to be surfaced (logged, reported -
/// whatever the consuming node does) rather than four scattered indices, so
/// whoever wires up the real robot has one place to check it against.
extern const std::array<const char *, 4> WHEEL_CORNERS;

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct WheelTargets
{
  std::array<double, 4> steer{{0.0, 0.0, 0.0, 0.0}};   ///< radians
  std::array<double, 4> spin{{0.0, 0.0, 0.0, 0.0}};    ///< radians/second
};

/// The achieved body-frame velocity for this tick: eta_dot_constrained,
/// straight from the model, before it gets rotated into the world frame to
/// integrate the pose. This is what a consumer expecting a body-frame
/// cmd_vel (e.g. gazebo_ros_planar_move, which applies its own heading)
/// needs - differencing the world-frame pose instead would hand it a
/// velocity already rotated once, then have it rotated a second time.
struct Velocity2D
{
  double vx{0.0};        ///< m/s, body frame
  double vy{0.0};        ///< m/s, body frame
  double yaw_rate{0.0};  ///< rad/s
};

/// The rover's kinematics with no ROS attached.
///
/// A kinematic simulation has no encoders, so the measured steering angle and
/// rate fed back into the model are its own previous outputs. That closes the
/// loop the way the rover's hardware would when it tracks perfectly, which is
/// the assumption this whole simulation rests on.
///
/// The model is the rover's own: Simulink 2.42, grt target, R2023a, with
/// Asterope's geometry fed in through hParams (../../vendor242/VENDOR242.md).
/// The simulation runs the arithmetic the wheels obey, which is what makes a
/// disagreement between sim and rover a real disagreement — and what makes
/// SP10's feasibility clamp trustworthy.
class SimIkStepper
{
public:
  /// `ts` defaults to the rover's own IK period; see asterope_params.hpp.
  explicit SimIkStepper(double ts = kIkTimestepSeconds);

  /// One tick of `ts` seconds against a body-frame velocity command.
  void step(double vx, double vy, double yaw_rate);

  const WheelTargets & targets() const {return targets_;}
  const Pose2D & pose() const {return pose_;}
  /// Replaces the integrated pose with one from outside (localisation).
  ///
  /// Not a blend and not a correction: what this replaces is dead
  /// reckoning, which is the thing localisation exists to stop trusting.
  /// The wheel and steering state is untouched, so the wheels keep turning
  /// from /manual_twist while the body goes where the rover really is.
  void set_pose(const Pose2D & pose) {pose_ = pose;}
  bool indirect_mode() const {return indirect_mode_;}
  std::array<double, 2> feasible_icr() const {return feasible_icr_;}
  const Velocity2D & achieved_velocity() const {return achieved_velocity_;}

private:
  double ts_;
  kinematics model_;
  // Model 2.42 takes the chassis geometry at runtime on hParams, so this
  // struct is persistent and carries it: setExternalInputs() copies the whole
  // struct by value on every tick, exactly as the rover's IkController does
  // with its own m_in. Filled once in the constructor.
  ExtU_kinematics_T in_{};
  WheelTargets targets_{};
  Pose2D pose_{};
  bool indirect_mode_{false};
  std::array<double, 2> feasible_icr_{{0.0, 0.0}};
  Velocity2D achieved_velocity_{};
};

}  // namespace navi_sim_ik

#endif  // NAVI_SIM_IK__SIM_IK_STEPPER_HPP_
