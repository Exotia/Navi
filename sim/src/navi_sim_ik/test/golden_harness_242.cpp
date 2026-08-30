// Generates the golden table pinned by test_ik_parity_242.cpp.
//
// Deliberately NOT a CMake target: it is a code generator, not a test, and
// keeping it out of the package build means it can never break the build,
// never install, and never accidentally link the 2.41 model. Build and run it
// by hand:
//
//   g++ -std=c++17 -O2 -w \
//     -I/home/ole/star/Navi/sim/src/navi_sim_ik/vendor242 \
//     -I/home/ole/star/Navi/sim/src/navi_sim_ik/include \
//     /home/ole/star/Navi/sim/src/navi_sim_ik/test/golden_harness_242.cpp \
//     /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242/*.cpp \
//     -o /tmp/golden_harness_242 && /tmp/golden_harness_242
//
// It compiles the SAME vendored sources the gtest links, with the SAME
// Asterope parameters, so the table it prints is the model's own output and
// not a transcription of anything. Regenerate it whenever vendor242/ or
// asterope_params.hpp changes - which, both being frozen, should be never
// without a deliberate re-vendor.

#include <cstdio>

#include "kinematics.h"
#include "navi_sim_ik/asterope_params.hpp"

namespace
{
// The commands the parity test pins. Chosen from
// docs/superpowers/specs/autonomy-plan.md §10: the manual cap
// (0.05 m/s, 0.1 rad/s), each autonomy speed stage (0.15, 0.30, 0.45 m/s),
// the wz ceiling (0.4 rad/s), and the reverse floor (vx_min >= -0.15 from §5).
// Plus zero, pure crab and pure point turn, which are the three degenerate
// cases a feasibility clamp will meet most often.
struct Command
{
  double vx;
  double vy;
  double wz;
};

constexpr Command kCommands[] = {
  {0.0, 0.0, 0.0},        // zero - nothing commanded
  {0.05, 0.0, 0.0},       // manual cap, pure translation
  {0.0, 0.05, 0.0},       // manual cap, pure crab
  {0.0, 0.0, 0.1},        // manual cap, pure point turn
  {0.05, 0.0, 0.1},       // manual cap, combined
  {0.15, 0.0, 0.0},       // autonomy stage 2
  {0.30, 0.0, 0.2},       // autonomy stage 3, combined
  {0.45, 0.0, 0.4},       // autonomy stage 4 at the wz ceiling
  {0.0, 0.0, 0.4},        // point turn at the wz ceiling
  {-0.15, 0.0, 0.0},      // the reverse floor
};

constexpr int kSteps = 100;   // 6 s at 0.06 s - long enough to settle
}  // namespace

int main()
{
  for (const Command & c : kCommands) {
    kinematics model;
    model.initialize();

    ExtU_kinematics_T in{};
    in.TS = navi_sim_ik::kIkTimestepSeconds;
    in.beta_dot_max = navi_sim_ik::kBetaDotMax;
    in.beta_ddot_max = navi_sim_ik::kBetaDdotMax;
    in.acceleration_factor = navi_sim_ik::kAccelerationFactor;
    for (int i = 0; i < 8; ++i) {
      in.hParams[i] = navi_sim_ik::kAsteropeHParams[i];
    }
    in.VX_out = c.vx;
    in.VY_out = c.vy;
    in.U = c.wz;   // rad/s, ROS sign - no negation, no degrees

    // Closed loop with the model's own outputs as its next measurement: a
    // kinematic simulation has no encoders, and this is the loop the rover's
    // hardware closes when it tracks perfectly.
    for (int k = 0; k < kSteps; ++k) {
      model.setExternalInputs(&in);
      model.step();
      const ExtY_kinematics_T & out = model.getExternalOutputs();
      for (int w = 0; w < 4; ++w) {
        in.beta_hat[w] = out.beta_next[w];
        in.beta_dot_hat[w] = out.Beta_dot[w];
      }
    }

    const ExtY_kinematics_T & out = model.getExternalOutputs();
    // %.17g round-trips a double exactly.
    std::printf(
      "  {{%g, %g, %g},\n"
      "   {%.17g, %.17g, %.17g, %.17g},\n"
      "   {%.17g, %.17g, %.17g, %.17g},\n"
      "   {%.17g, %.17g, %.17g}},\n",
      c.vx, c.vy, c.wz,
      out.beta_next[0], out.beta_next[1], out.beta_next[2], out.beta_next[3],
      out.omega[0], out.omega[1], out.omega[2], out.omega[3],
      out.eta_dot_constrained[0], out.eta_dot_constrained[1],
      out.eta_dot_constrained[2]);
  }
  return 0;
}
