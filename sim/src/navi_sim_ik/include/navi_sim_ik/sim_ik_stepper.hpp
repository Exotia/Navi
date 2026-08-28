#ifndef NAVI_SIM_IK__SIM_IK_STEPPER_HPP_
#define NAVI_SIM_IK__SIM_IK_STEPPER_HPP_

#include <array>

#include "kinematics.h"

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

/// The rover's kinematics with no ROS attached.
///
/// A kinematic simulation has no encoders, so the measured steering angle and
/// rate fed back into the model are its own previous outputs. That closes the
/// loop the way the rover's hardware would when it tracks perfectly, which is
/// the assumption this whole simulation rests on.
class SimIkStepper
{
public:
  explicit SimIkStepper(double ts = 0.06);

  /// One tick of `ts` seconds against a body-frame velocity command.
  void step(double vx, double vy, double yaw_rate);

  const WheelTargets & targets() const {return targets_;}
  const Pose2D & pose() const {return pose_;}
  bool indirect_mode() const {return indirect_mode_;}
  std::array<double, 2> feasible_icr() const {return feasible_icr_;}

private:
  double ts_;
  kinematics model_;
  kinematics::ExternalInputs in_{};
  WheelTargets targets_{};
  Pose2D pose_{};
  bool indirect_mode_{false};
  std::array<double, 2> feasible_icr_{{0.0, 0.0}};
};

}  // namespace navi_sim_ik

#endif  // NAVI_SIM_IK__SIM_IK_STEPPER_HPP_
