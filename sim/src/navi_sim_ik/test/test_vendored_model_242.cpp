#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "kinematics.h"

// Proves the 2.42 copy is complete and usable: it compiles, initialises,
// accepts geometry through the hParams inport, and steps. Not a test of the
// control law, which belongs to whoever regenerates the model upstream - this
// is the guard that says the copy is complete.
//
// The eight geometry numbers are inline here only for this first task; Task 2
// replaces them with the shared header, so the rover's values live in exactly
// one place.
namespace
{
void set_asterope_geometry(ExtU_kinematics_T & in)
{
  // Declared float, then widened. The rover holds these as `const float` in
  // RoverParameters.h and hands them to the model through a
  // std::vector<float>, so the double the model actually sees is a widened
  // float, not the decimal literal. Writing them as double here would change
  // the arithmetic in the 8th decimal place - which is exactly the kind of
  // difference this whole sub-project exists to eliminate.
  const float wheel[8] = {
    0.45527f, -0.44385f,
    0.45527f, 0.44385f,
    -0.45527f, 0.44285f,   // wheel3y: 0.44285, not 0.44385 - see VENDOR242.md
    -0.45527f, -0.44385f};
  for (int i = 0; i < 8; ++i) {
    in.hParams[i] = wheel[i];
  }
}

void set_ik_limits(ExtU_kinematics_T & in)
{
  in.TS = 0.06;
  in.beta_dot_max = 1.5;
  in.beta_ddot_max = 250.0;
  in.acceleration_factor = 3.0;
}

// One closed-loop run from a fresh model. A kinematic simulation has no
// encoders, so the measured steering angle and rate fed back in are the
// model's own previous outputs - the same loop closure the rover's hardware
// makes when it tracks perfectly.
void run(ExtU_kinematics_T & in, kinematics & model, int steps)
{
  for (int i = 0; i < steps; ++i) {
    model.setExternalInputs(&in);
    model.step();
    const ExtY_kinematics_T & out = model.getExternalOutputs();
    for (int w = 0; w < 4; ++w) {
      in.beta_hat[w] = out.beta_next[w];
      in.beta_dot_hat[w] = out.Beta_dot[w];
    }
  }
}
}  // namespace

TEST(VendoredModel242, DrivingStraightAheadPointsEveryWheelForward)
{
  kinematics model;
  model.initialize();

  ExtU_kinematics_T in{};
  set_ik_limits(in);
  set_asterope_geometry(in);
  in.VX_out = 0.5;   // straight ahead, m/s, body frame
  in.VY_out = 0.0;
  in.U = 0.0;        // rad/s, ROS sign convention - see the plan's unit note

  run(in, model, 100);

  const ExtY_kinematics_T & out = model.getExternalOutputs();
  for (int w = 0; w < 4; ++w) {
    EXPECT_NEAR(out.beta_next[w], 0.0, 0.05) << "wheel " << w << " is not straight";
    EXPECT_GT(out.omega[w], 0.0) << "wheel " << w << " is not driving forward";
  }
}

TEST(VendoredModel242, TurningInPlaceSpreadsTheWheelAngles)
{
  kinematics model;
  model.initialize();

  ExtU_kinematics_T in{};
  set_ik_limits(in);
  set_asterope_geometry(in);
  in.VX_out = 0.0;
  in.VY_out = 0.0;
  in.U = 0.4;        // yaw only, rad/s - the spec's autonomy wz cap

  run(in, model, 200);

  const ExtY_kinematics_T & out = model.getExternalOutputs();
  const double lo = *std::min_element(out.beta_next, out.beta_next + 4);
  const double hi = *std::max_element(out.beta_next, out.beta_next + 4);
  // Spinning about a point puts the four wheels on tangents to a circle, so
  // they cannot all point the same way.
  EXPECT_GT(hi - lo, 0.2) << "all four wheels point the same way while turning in place";
}

TEST(VendoredModel242, GeometryOnHParamsActuallyReachesTheArithmetic)
{
  // The failure this guards against is the quiet one: hParams set, ignored,
  // and the model still running whatever geometry it was generated with. If
  // that happened, every other test here would still pass and the simulation
  // would be a Merope again while claiming to be an Asterope. So: run the
  // same command through Asterope's geometry and through Merope's (from the
  // #if MEROPE block of RoverParameters.h) and require the outputs to differ.
  const float asterope[8] = {
    0.45527f, -0.44385f, 0.45527f, 0.44385f,
    -0.45527f, 0.44285f, -0.45527f, -0.44385f};
  const float merope[8] = {
    0.404f, -0.285f, 0.404f, 0.285f,
    -0.404f, 0.285f, -0.404f, -0.285f};

  double omega_a = 0.0;
  double omega_m = 0.0;
  for (int which = 0; which < 2; ++which) {
    kinematics model;
    model.initialize();
    ExtU_kinematics_T in{};
    set_ik_limits(in);
    const float * h = (which == 0) ? asterope : merope;
    for (int i = 0; i < 8; ++i) {
      in.hParams[i] = h[i];
    }
    in.VX_out = 0.2;
    in.VY_out = 0.0;
    in.U = 0.3;
    run(in, model, 100);
    const double omega0 = model.getExternalOutputs().omega[0];
    (which == 0 ? omega_a : omega_m) = omega0;
  }

  EXPECT_GT(std::abs(omega_a - omega_m), 0.1)
    << "the two geometries produced the same wheel speed - hParams is not live";
}
