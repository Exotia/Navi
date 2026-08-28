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
}

void SimIkStepper::step(double vx, double vy, double yaw_rate)
{
  in_.VX_out = vx;
  in_.VY_out = vy;
  in_.U_p = yaw_rate;

  model_.setExternalInputs(&in_);
  model_.step();
  const auto & out = model_.getExternalOutputs();

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

  // Integrate what the controller could actually deliver, not what was asked
  // for. eta_dot_constrained is the request after the ICR feasibility limits
  // have been applied, so a command the geometry cannot satisfy moves the
  // rover the way the rover would move rather than the way it was told to.
  const double achieved_vx = out.eta_dot_constrained[0];
  const double achieved_vy = out.eta_dot_constrained[1];
  const double achieved_yaw_rate = out.eta_dot_constrained[2];

  const double cos_yaw = std::cos(pose_.yaw);
  const double sin_yaw = std::sin(pose_.yaw);
  pose_.x += (achieved_vx * cos_yaw - achieved_vy * sin_yaw) * ts_;
  pose_.y += (achieved_vx * sin_yaw + achieved_vy * cos_yaw) * ts_;
  pose_.yaw += achieved_yaw_rate * ts_;
}

}  // namespace navi_sim_ik
