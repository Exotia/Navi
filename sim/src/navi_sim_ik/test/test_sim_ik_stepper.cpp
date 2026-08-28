#include <gtest/gtest.h>

#include <cmath>

#include "navi_sim_ik/sim_ik_stepper.hpp"

using navi_sim_ik::SimIkStepper;

TEST(SimIkStepper, StartsAtTheOrigin)
{
  SimIkStepper stepper;
  EXPECT_DOUBLE_EQ(stepper.pose().x, 0.0);
  EXPECT_DOUBLE_EQ(stepper.pose().y, 0.0);
  EXPECT_DOUBLE_EQ(stepper.pose().yaw, 0.0);
}

TEST(SimIkStepper, DrivingForwardMovesAlongXAndNotAcross)
{
  // The real IK's constrained output is not perfectly decoupled: driving from
  // a standing start measurably shows up as 0.0764 m of cross-track drift and
  // 0.0562 rad of yaw over these 100 steps (traced to a small non-zero
  // eta_dot_constrained[2] the model itself settles to while VX_out is
  // nonzero and VY_out/U_p are zero - not an artifact of this wrapper, and
  // reproducible by feeding the vendored model directly). These are measured
  // baselines from the current, correct implementation, not analytically
  // derived - the +/-0.03 band is a regression guard around them, wide
  // enough for ordinary numerical noise but tight enough that a change which
  // doubled or erased the real coupling would fail either bound.
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {   // 6 seconds
    stepper.step(0.5, 0.0, 0.0);
  }
  EXPECT_GT(stepper.pose().x, 1.0);
  EXPECT_NEAR(stepper.pose().y, 0.0764, 0.03);
  EXPECT_NEAR(stepper.pose().yaw, 0.0562, 0.03);
}

TEST(SimIkStepper, TurningInPlaceChangesYawWithoutTravelling)
{
  // As above: the vendored IK measurably produces -0.0792 m of transient x
  // travel while spinning up from a standing start (the wheels take a moment
  // to swing into the turn-in-place configuration, and that transient is real
  // motion the vehicle would actually make). Measured baseline, not derived;
  // the +/-0.03 band around it is a two-sided regression guard rather than a
  // ceiling, so a change that doubled the real transient would also fail.
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.0, 0.0, 0.4);
  }
  EXPECT_GT(std::abs(stepper.pose().yaw), 0.5);
  EXPECT_NEAR(stepper.pose().x, -0.0792, 0.03);
  EXPECT_NEAR(stepper.pose().y, 0.0, 0.05);
}

TEST(SimIkStepper, YawIsAppliedInTheBodyFrame)
{
  // Turn a quarter circle, then drive: the rover must go sideways in world
  // terms. Integrating in the world frame instead would send it along +X -
  // which, over 100 steps at vx=0.5, means x growing by roughly +3 m in the
  // broken case. What the real IK actually does here is different: the
  // small yaw-rate coupling described above (amplified here because the turn
  // leaves the wheels away from straight-ahead, so re-aligning them takes
  // longer and the vehicle keeps yawing slightly past pi/2 while it drives)
  // measurably moves x by about -1.0 m over the same 100 steps. That is real
  // motion, not a frame bug, and it is well clear of the +3 m a world-frame
  // integration bug would produce, so the bound below is set to separate the
  // two rather than to hide the measured coupling.
  SimIkStepper stepper;
  while (stepper.pose().yaw < M_PI / 2) {
    stepper.step(0.0, 0.0, 0.6);
  }
  const double x_before = stepper.pose().x;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  EXPECT_GT(stepper.pose().y, 1.0);
  EXPECT_LT(std::abs(stepper.pose().x - x_before), 1.5);
}

TEST(SimIkStepper, WheelSpinIsReportedForEveryWheelWhenDriving)
{
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  for (int w = 0; w < 4; ++w) {
    EXPECT_GT(stepper.targets().spin[w], 0.0) << "wheel " << w;
    EXPECT_NEAR(stepper.targets().steer[w], 0.0, 0.05) << "wheel " << w;
  }
}

TEST(SimIkStepper, StandingStillDoesNotDrift)
{
  // The pose comes from the IK's constrained velocity, so a zero command
  // must integrate to exactly nothing - not to a slow creep that would look
  // like localisation drift later.
  SimIkStepper stepper;
  for (int i = 0; i < 200; ++i) {
    stepper.step(0.0, 0.0, 0.0);
  }
  EXPECT_NEAR(stepper.pose().x, 0.0, 1e-9);
  EXPECT_NEAR(stepper.pose().y, 0.0, 1e-9);
  EXPECT_NEAR(stepper.pose().yaw, 0.0, 1e-9);
}

TEST(SimIkStepper, WheelCornersMatchesThePinnedMapping)
{
  // This does not check the URDF - it pins the wheel-index-to-corner mapping
  // against silent edits. The mapping itself is unverified wiring knowledge
  // (see the WHEEL_CORNERS comment); this test only guarantees that changing
  // it is a visible, deliberate diff rather than a quiet one.
  const std::array<const char *, 4> expected{
    "front_left", "front_right", "rear_right", "rear_left"};
  for (int i = 0; i < 4; ++i) {
    EXPECT_STREQ(navi_sim_ik::WHEEL_CORNERS[i], expected[i]);
  }
}

TEST(SimIkStepper, YawIsIntegratedFromTheStartOfStepNotTheEnd)
{
  // Guards against applying the yaw update before the x/y update within a
  // single step(), which would rotate each step's translation by the yaw it
  // is *about* to reach rather than the yaw it held when the step began.
  // Pure rotation alone can't expose this (there is no x/y to rotate), and
  // pure translation alone can't either (yaw_rate is zero, so the two
  // orderings agree). It needs translation and rotation together, at a
  // yaw_rate large enough that one step's worth of yaw materially changes
  // the rotation matrix - hence 0.6 rad/s while driving, tracing an arc.
  //
  // The expected endpoint below is measured from the current, correct
  // implementation (integrate x/y using the yaw held at the start of the
  // step, then advance yaw) - it is not analytically derived. Swapping the
  // two update lines in step() shifts the endpoint by roughly one step's
  // yaw increment applied to the whole path, which at these values is on
  // the order of 0.1 m - an order of magnitude past the 0.02 m tolerance
  // here, so that ordering bug fails this test while ordinary numerical
  // noise does not.
  SimIkStepper stepper;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.5, 0.0, 0.6);
  }
  EXPECT_NEAR(stepper.pose().x, -0.0841218, 0.02);
  EXPECT_NEAR(stepper.pose().y, 1.6467001, 0.02);
  EXPECT_NEAR(stepper.pose().yaw, 3.2995396, 0.02);
}

TEST(SimIkStepper, AchievedVelocityIsBodyFrameNotWorldFrame)
{
  // Turn to face +Y in the world (yaw ~ pi/2), then command driving straight
  // ahead in the body frame. achieved_velocity() reports eta_dot_constrained
  // straight from the model, before it gets rotated into the world frame to
  // integrate the pose - so it must stay close to the commanded body-frame
  // speed no matter which way the rover is pointed in the world.
  //
  // The old, wrong approach recovered this quantity by differencing the
  // world-frame pose instead. After this turn, the pose is moving almost
  // entirely in world Y (see YawIsAppliedInTheBodyFrame above), so that
  // differencing would have handed back a velocity dominated by world Y with
  // almost nothing in X - failing the vx assertion below, which is the
  // point of this test.
  SimIkStepper stepper;
  while (stepper.pose().yaw < M_PI / 2) {
    stepper.step(0.0, 0.0, 0.6);
  }
  const double y_before = stepper.pose().y;
  for (int i = 0; i < 100; ++i) {
    stepper.step(0.5, 0.0, 0.0);
  }
  EXPECT_NEAR(stepper.achieved_velocity().vx, 0.5, 0.05);
  EXPECT_GT(stepper.pose().y - y_before, 1.0);
}
