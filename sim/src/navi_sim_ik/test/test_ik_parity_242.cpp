#include <gtest/gtest.h>

#include <string>

#include "kinematics.h"
#include "navi_sim_ik/asterope_params.hpp"

// Numeric parity between this simulation and the rover.
//
// The whole of SP4 comes down to this file. The simulation and the rover run
// the same generated model (2.42, grt, R2023a) with the same geometry on
// hParams, so for a given command they must produce the same wheel angles and
// the same wheel speeds - not similar ones. If they ever diverge, SP10's
// twist_shaper is clamping against arithmetic the wheels do not obey, and a
// command it calls feasible may not be.
//
// The expected values are NOT hand-derived and are NOT transcribed from
// anywhere. They are the output of test/golden_harness_242.cpp, which compiles
// the same vendored sources with the same parameters; that file's header
// comment carries the exact command that regenerates them.
//
// Tolerance is 1e-9 absolute rather than exact equality. The arithmetic is
// deterministic double arithmetic from identical sources, so on one machine it
// is bit-identical - but the harness is built with a plain `g++ -O2` line and
// the test through colcon's flags, and pinning to the last bit would turn a
// compiler-flag change into a mysterious failure about the rover's kinematics.
// 1e-9 rad is 6e-8 degrees of steering: nine orders of magnitude below
// anything the chassis can resolve, and still tight enough that a changed
// geometry value (the smallest of which, wheel3y's 1 mm, moves these outputs
// by ~1e-4) fails every row.
namespace
{
constexpr double kTolerance = 1e-9;
constexpr int kSteps = 100;

struct GoldenCase
{
  double command[3];               // vx (m/s), vy (m/s), wz (rad/s)
  double beta_next[4];             // rad
  double omega[4];                 // rad/s
  double eta_dot_constrained[3];   // m/s, m/s, rad/s - body frame
};

// GENERATED - do not hand-edit. See test/golden_harness_242.cpp.
constexpr GoldenCase kGolden[] = {
  {{0, 0, 0},
   {0.79809876803175994, -3.9396914216215531, -2.3423664859124607, -0.79809876803175961},
   {0, 0, 0, 0},
   {0, 0, 0}},
  {{0.05, 0, 0},
   {0.044118250000424908, 0.048267420445780695, -0.048262307593531784, -0.044118250000424908},
   {0.41397406423199778, 0.37881471249317711, 0.37885431601233355, 0.41397406370459522},
   {0.049497803865977526, 7.4738577789389793e-10, 0.0049560546903008457}},
  {{0, 0.05, 0},
   {1.527830876391, -4.6694235299807936, -4.6653794817118781, 1.523680830983146},
   {0.41439776941869449, 0.4143977582125809, 0.37835245334085638, 0.37835430508430273},
   {1.6306069845785876e-08, 0.049498115954540581, 0.0049529747612972845}},
  {{0, 0, 0.1},
   {0.79809876803175994, -3.9396914216215531, -2.3423664859124607, -0.79809876803175961},
   {0.50866010372488801, 0.50866010372488801, 0.50810196173178668, 0.50866010372488801},
   {1.9318465963213229e-18, 5.9415969938417866e-18, 0.099999999706524831}},
  {{0.05, 0, 0.1},
   {0.45951483345555344, 1.4999111199620203, -1.4977259450269227, -0.45951483345555344},
   {0.82883289482965683, 0.36850254516410469, 0.36856092282013592, 0.82883297143647894},
   {0.048064574766342488, -1.0795451155469119e-08, 0.10092160283642451}},
  {{0.15, 0, 0},
   {0.016677253341802478, 0.017237733543641554, -0.017237080955472717, -0.016677253341802256},
   {1.2180952161150813, 1.1789568047197023, 1.1790008948626756, 1.2180952160376251},
   {0.14979473048526942, 2.9029069961004752e-10, 0.0055119833793934236}},
  {{0.3, 0, 0.2},
   {0.23436783022064667, 0.42012022299438767, -0.41975511007149335, -0.23436783022064644},
   {3.1908061562924468, 1.8168947881122981, 1.818380840333935, 3.1908061428113723},
   {0.29766030543366373, 3.6281868708401557e-09, 0.20342296404014232}},
  {{0.45, 0, 0.4},
   {0.28485658628707755, 0.59887303764294231, -0.59817575498298825, -0.28485658628707733},
   {5.2205171213089425, 2.602602620702601, 2.6052648746969327, 5.2205171100660728},
   {0.44749476253905845, 2.5004619544525082e-09, 0.40278313642298352}},
  {{0, 0, 0.4},
   {0.79809876803175994, -3.9396914216215531, -2.3423664859124607, -0.79809876803175961},
   {2.0346404148995521, 2.0346404148995521, 2.0324078469271467, 2.0346404148995521},
   {7.7273863852852915e-18, 2.3766387975367146e-17, 0.39999999882609932}},
  {{-0.15, 0, 0},
   {-3.1588303871334347, -3.1582699069315954, -3.1249147893579847, -3.1243549200461516},
   {1.178956795394329, 1.2180952241343339, 1.2180511335607991, 1.178956795469273},
   {-0.14979473039428881, -2.717424442849518e-10, 0.0055119858224476686}},
};

std::string label(const GoldenCase & g)
{
  return "vx=" + std::to_string(g.command[0]) +
         " vy=" + std::to_string(g.command[1]) +
         " wz=" + std::to_string(g.command[2]);
}
}  // namespace

TEST(IkParity242, EveryCommandReproducesTheRoversArithmetic)
{
  ASSERT_GT(sizeof(kGolden) / sizeof(kGolden[0]), 0u)
    << "the golden table is empty - run test/golden_harness_242.cpp";

  for (const GoldenCase & g : kGolden) {
    SCOPED_TRACE(label(g));

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
    in.VX_out = g.command[0];
    in.VY_out = g.command[1];
    // rad/s, ROS sign convention. bema_bridge.py sends -degrees(angular.z)
    // and BemaServer::drive() computes -pi*w/180, so the two negations cancel
    // and the model's U is angular.z in rad/s. Converting again here would
    // be the double-conversion bug this comment exists to prevent.
    in.U = g.command[2];

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
    for (int w = 0; w < 4; ++w) {
      EXPECT_NEAR(out.beta_next[w], g.beta_next[w], kTolerance)
        << "steering angle, wheel " << w;
      EXPECT_NEAR(out.omega[w], g.omega[w], kTolerance)
        << "wheel speed, wheel " << w;
    }
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(out.eta_dot_constrained[i], g.eta_dot_constrained[i], kTolerance)
        << "constrained body velocity, component " << i;
    }
  }
}

TEST(IkParity242, TheZeroCommandProducesNoMotionAtAll)
{
  // Called out separately from the table because it is the one row whose
  // correctness is not a matter of matching a generated number: a rover that
  // creeps when told to stand still is a runaway, and no tolerance band should
  // ever be what stands between us and noticing.
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

  for (int k = 0; k < 200; ++k) {
    model.setExternalInputs(&in);
    model.step();
    const ExtY_kinematics_T & out = model.getExternalOutputs();
    for (int w = 0; w < 4; ++w) {
      in.beta_hat[w] = out.beta_next[w];
      in.beta_dot_hat[w] = out.Beta_dot[w];
    }
  }

  const ExtY_kinematics_T & out = model.getExternalOutputs();
  for (int w = 0; w < 4; ++w) {
    EXPECT_DOUBLE_EQ(out.omega[w], 0.0) << "wheel " << w << " spins at rest";
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(out.eta_dot_constrained[i], 0.0)
      << "the body moves at rest, component " << i;
  }
}
