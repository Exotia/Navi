#include <gtest/gtest.h>

#include <cmath>

#include "kinematics.h"

// Proves the vendored model builds, initialises and steps. Not a test of the
// control law, which belongs to whoever regenerates it - this is the guard
// that says the copy is complete and usable.
TEST(VendoredModel, DrivingStraightAheadPointsEveryWheelForward)
{
  kinematics model;
  model.initialize();

  kinematics::ExternalInputs in{};
  in.TS = 0.06;
  in.VX_out = 0.5;   // straight ahead
  in.VY_out = 0.0;
  in.U_p = 0.0;      // no yaw

  for (int i = 0; i < 100; ++i) {
    model.setExternalInputs(&in);
    model.step();
    const auto & out = model.getExternalOutputs();
    for (int w = 0; w < 4; ++w) {
      in.beta_hat[w] = out.beta_next[w];
      in.beta_dot_hat[w] = out.Beta_dot[w];
    }
  }

  const auto & out = model.getExternalOutputs();
  for (int w = 0; w < 4; ++w) {
    EXPECT_NEAR(out.beta_next[w], 0.0, 0.05) << "wheel " << w << " is not straight";
    EXPECT_GT(out.omega[w], 0.0) << "wheel " << w << " is not driving forward";
  }
}

TEST(VendoredModel, TurningInPlaceSpreadsTheWheelAngles)
{
  kinematics model;
  model.initialize();

  kinematics::ExternalInputs in{};
  in.TS = 0.06;
  in.VX_out = 0.0;
  in.VY_out = 0.0;
  in.U_p = 0.4;      // yaw only

  for (int i = 0; i < 200; ++i) {
    model.setExternalInputs(&in);
    model.step();
    const auto & out = model.getExternalOutputs();
    for (int w = 0; w < 4; ++w) {
      in.beta_hat[w] = out.beta_next[w];
      in.beta_dot_hat[w] = out.Beta_dot[w];
    }
  }

  const auto & out = model.getExternalOutputs();
  double lo = out.beta_next[0], hi = out.beta_next[0];
  for (int w = 1; w < 4; ++w) {
    lo = std::min(lo, out.beta_next[w]);
    hi = std::max(hi, out.beta_next[w]);
  }
  // Spinning about a point puts the four wheels on tangents to a circle, so
  // they cannot all point the same way.
  EXPECT_GT(hi - lo, 0.2) << "all four wheels point the same way while turning in place";
}
