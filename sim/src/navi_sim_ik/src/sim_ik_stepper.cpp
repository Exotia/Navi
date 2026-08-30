#include "navi_sim_ik/sim_ik_stepper.hpp"

#include <cmath>

namespace navi_sim_ik
{

const std::array<const char *, 4> WHEEL_CORNERS{
  {"front_left", "front_right", "rear_right", "rear_left"}};

SimIkStepper::SimIkStepper(double ts)
: ts_(ts)
{
  model_.initialize();
  in_.TS = ts_;

  // The rover's own constants, from IkController's constructor. They are
  // inports on this model rather than baked-in parameters, so they have to be
  // supplied - a default-constructed ExtU_kinematics_T would hand the model
  // beta_dot_max = 0 and freeze the steering solid.
  in_.beta_dot_max = kBetaDotMax;
  in_.beta_ddot_max = kBetaDdotMax;
  in_.acceleration_factor = kAccelerationFactor;

  // Asterope's chassis geometry. Element-by-element from a std::array<float>
  // into a real_T[8], which is the same float-to-double widening the rover
  // performs when IkController copies its std::vector<float> - see the note
  // in asterope_params.hpp about why that matters.
  for (int i = 0; i < 8; ++i) {
    in_.hParams[i] = kAsteropeHParams[i];
  }
}

void SimIkStepper::step(double vx, double vy, double yaw_rate)
{
  in_.VX_out = vx;
  in_.VY_out = vy;
  // rad/s, straight through. The rover reaches the same number by two
  // negations that cancel: bema_bridge.py sends w = -degrees(angular.z) over
  // RPC, and BemaServer::drive() computes u = -pi*w/180. Converting to
  // degrees or flipping the sign here would apply that pair a second time and
  // spin the simulated rover backwards at a 57th of the commanded rate.
  in_.U = yaw_rate;

  model_.setExternalInputs(&in_);
  model_.step();
  const ExtY_kinematics_T & out = model_.getExternalOutputs();

  for (int w = 0; w < 4; ++w) {
    targets_.steer[w] = out.beta_next[w];
    targets_.spin[w] = out.omega[w];
    // No encoders in a kinematic simulation: the model's own output is what
    // it measures next tick.
    in_.beta_hat[w] = out.beta_next[w];
    in_.beta_dot_hat[w] = out.Beta_dot[w];
  }

  indirect_mode_ = out.indirect_mode;
  feasible_icr_ = {out.feasable_ICR[0], out.feasable_ICR[1]};

  // What the controller could actually deliver, not what was asked for.
  // eta_dot_constrained is the request after the ICR feasibility limits have
  // been applied, so a command the geometry cannot satisfy moves the rover
  // the way the rover would move rather than the way it was told to. This is
  // the body-frame velocity: expose it as-is for consumers that want the
  // body frame (e.g. a cmd_vel a planar-move plugin will rotate itself), and
  // separately rotate it into the world frame below to integrate the pose.
  achieved_velocity_.vx = out.eta_dot_constrained[0];
  achieved_velocity_.vy = out.eta_dot_constrained[1];
  achieved_velocity_.yaw_rate = out.eta_dot_constrained[2];

  const double cos_yaw = std::cos(pose_.yaw);
  const double sin_yaw = std::sin(pose_.yaw);
  pose_.x += (achieved_velocity_.vx * cos_yaw - achieved_velocity_.vy * sin_yaw) * ts_;
  pose_.y += (achieved_velocity_.vx * sin_yaw + achieved_velocity_.vy * cos_yaw) * ts_;
  pose_.yaw += achieved_velocity_.yaw_rate * ts_;
}

}  // namespace navi_sim_ik
