#include <gtest/gtest.h>

#include <string>

#include "navi_sim_ik/external_pose.hpp"

using navi_sim_ik::ExternalPoseGate;
using navi_sim_ik::localization_state;

// The exact payload localization_status publishes (sub-project 1). Pinned
// here as a whole string rather than a fragment: this is the wire format
// this node reads, and a change to it should break this file.
constexpr char kOkStatus[] =
  "{\"state\": \"OK\", \"seconds_since_ok\": 0.0, \"source\": \"zed_vio\", "
  "\"distance_travelled\": 12.5, \"mount_offset_verified\": true}";
constexpr char kSearchingStatus[] =
  "{\"state\": \"SEARCHING\", \"seconds_since_ok\": 4.2, \"source\": \"zed_vio\", "
  "\"distance_travelled\": 12.5, \"mount_offset_verified\": true}";

TEST(LocalizationState, ReadsTheStateOutOfARealStatusPayload)
{
  EXPECT_EQ(localization_state(kOkStatus), "OK");
  EXPECT_EQ(localization_state(kSearchingStatus), "SEARCHING");
}

TEST(LocalizationState, EmptyWhenThereIsNoStateToRead)
{
  // Anything that is not a status with a string state must read as "not OK",
  // and "" is not OK. A payload this node cannot understand is exactly when
  // it must not move the model.
  EXPECT_EQ(localization_state(""), "");
  EXPECT_EQ(localization_state("not json at all"), "");
  EXPECT_EQ(localization_state("{\"source\": \"zed_vio\"}"), "");
  // A non-string state must not be answered with the next quoted thing in
  // the payload, which would return "zed_vio" and read as an unknown state.
  EXPECT_EQ(localization_state("{\"state\": 3, \"source\": \"zed_vio\"}"), "");
}

TEST(ExternalPoseGate, RefusesEverythingBeforeAnyStatusArrives)
{
  // Startup order is not guaranteed: a pose can arrive before the first 2 Hz
  // status. Moving the model on a pose whose health is unknown is the one
  // thing this gate exists to prevent.
  ExternalPoseGate gate;
  EXPECT_FALSE(gate.ok());
  EXPECT_FALSE(gate.accept(0.0));
}

TEST(ExternalPoseGate, AcceptsWhileLocalisationIsOk)
{
  ExternalPoseGate gate;
  gate.set_state("OK");
  EXPECT_TRUE(gate.ok());
  EXPECT_TRUE(gate.accept(100.0));
}

TEST(ExternalPoseGate, HoldsStillWhileSearchingAndWhileOff)
{
  ExternalPoseGate gate;
  gate.set_state("OK");
  ASSERT_TRUE(gate.accept(100.0));

  gate.set_state("SEARCHING");
  EXPECT_FALSE(gate.accept(101.0));
  gate.set_state("OFF");
  EXPECT_FALSE(gate.accept(102.0));
}

TEST(ExternalPoseGate, RecoversOnItsOwnWhenLocalisationComesBack)
{
  // Recovery is automatic when the SDK re-acquires - nothing restarts, and
  // nothing has to be pressed.
  ExternalPoseGate gate;
  gate.set_state("SEARCHING");
  EXPECT_FALSE(gate.accept(100.0));

  gate.set_state("OK");
  EXPECT_TRUE(gate.accept(101.0));
}

TEST(ExternalPoseGate, CapsTheRateAtThirtyHertz)
{
  // /localization/pose arrives at the wrapper's 30 Hz today, but nothing
  // guarantees that: a faster publisher would put a service call and a
  // physics-thread write on Gazebo for every message. The cap is on this
  // side because it is the side that knows.
  ExternalPoseGate gate(30.0);
  gate.set_state("OK");
  ASSERT_TRUE(gate.accept(100.0));

  EXPECT_FALSE(gate.accept(100.010));    // 100 Hz worth of poses
  EXPECT_FALSE(gate.accept(100.030));    // still inside 1/30 s
  EXPECT_TRUE(gate.accept(100.034));     // 1/30 s = 0.0333 s has passed
}

TEST(ExternalPoseGate, ARefusedPoseDoesNotRestartTheRateWindow)
{
  // If a refusal moved the window, a fast publisher would starve the gate
  // forever: every message would land inside a window its predecessor just
  // reset, and the model would never move at all.
  ExternalPoseGate gate(30.0);
  gate.set_state("OK");
  ASSERT_TRUE(gate.accept(100.0));
  for (double t = 100.005; t < 100.033; t += 0.005) {
    EXPECT_FALSE(gate.accept(t));
  }
  EXPECT_TRUE(gate.accept(100.034));
}

TEST(ExternalPoseGate, TheFirstPoseIsNeverRateLimited)
{
  // Whatever "now" happens to be at startup, the first accepted pose must
  // go through: a steady clock starts wherever it starts, and comparing it
  // against a zero-initialised last-applied time would either pass by luck
  // or block for as long as the machine has been up.
  ExternalPoseGate gate(30.0);
  gate.set_state("OK");
  EXPECT_TRUE(gate.accept(1.0e6));
}
