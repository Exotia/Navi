#include <gtest/gtest.h>

#include <type_traits>

#include "navi_sim_ik/asterope_params.hpp"

using namespace navi_sim_ik;  // NOLINT - a constants header, nothing else

TEST(AsteropeParams, HParamsAreOrderedTheWayTheRoverPacksThem)
{
  // BemaServer.cpp:32 constructs IkController with
  // {wheel1x, wheel1y, wheel2x, wheel2y, wheel3x, wheel3y, wheel4x, wheel4y}
  // and IkController's constructor copies that vector straight into
  // hParams[0..7]. Getting the order wrong here mirrors the chassis about the
  // wrong axis and still produces plausible-looking wheel angles.
  EXPECT_FLOAT_EQ(kAsteropeHParams[0], kWheel1X);
  EXPECT_FLOAT_EQ(kAsteropeHParams[1], kWheel1Y);
  EXPECT_FLOAT_EQ(kAsteropeHParams[2], kWheel2X);
  EXPECT_FLOAT_EQ(kAsteropeHParams[3], kWheel2Y);
  EXPECT_FLOAT_EQ(kAsteropeHParams[4], kWheel3X);
  EXPECT_FLOAT_EQ(kAsteropeHParams[5], kWheel3Y);
  EXPECT_FLOAT_EQ(kAsteropeHParams[6], kWheel4X);
  EXPECT_FLOAT_EQ(kAsteropeHParams[7], kWheel4Y);
}

TEST(AsteropeParams, TheValuesAreTheRoversOwn)
{
  EXPECT_FLOAT_EQ(kWheel1X, 0.45527f);
  EXPECT_FLOAT_EQ(kWheel1Y, -0.44385f);
  EXPECT_FLOAT_EQ(kWheel2X, 0.45527f);
  EXPECT_FLOAT_EQ(kWheel2Y, 0.44385f);
  EXPECT_FLOAT_EQ(kWheel3X, -0.45527f);
  EXPECT_FLOAT_EQ(kWheel3Y, 0.44285f);
  EXPECT_FLOAT_EQ(kWheel4X, -0.45527f);
  EXPECT_FLOAT_EQ(kWheel4Y, -0.44385f);
}

TEST(AsteropeParams, Wheel3YKeepsTheRoversOneMillimetreAsymmetry)
{
  // RoverParameters.h has wheel3y = 0.44285 where wheels 1, 2 and 4 use
  // 0.44385. Almost certainly an upstream typo - and deliberately kept, so
  // that this simulation's arithmetic is the arithmetic the rover executes
  // rather than the arithmetic it ought to. This test exists so that
  // "tidying" it is a visible, deliberate diff instead of a quiet one. If it
  // is ever fixed upstream, fix it here in the same commit and regenerate
  // test_ik_parity_242.cpp's golden table.
  EXPECT_NE(kWheel3Y, kWheel2Y);
  EXPECT_NEAR(static_cast<double>(kWheel2Y - kWheel3Y), 0.001, 1e-6);
}

TEST(AsteropeParams, GeometryIsHeldAsFloatSoItWidensLikeTheRovers)
{
  // The rover holds these as `const float` and passes them through a
  // std::vector<float>, so what reaches hParams is a widened float. Declaring
  // them double here would change the arithmetic in the 8th decimal place -
  // small, invisible, and exactly the kind of "porting artifact" that
  // re-vendoring exists to eliminate.
  static_assert(
    std::is_same<decltype(kWheel1X), const float>::value,
    "geometry must be float, not double");
  const double widened = static_cast<double>(kWheel1X);
  EXPECT_DOUBLE_EQ(widened, static_cast<double>(0.45527f));
  EXPECT_NE(widened, 0.45527);   // the decimal literal is a different number
}

TEST(AsteropeParams, IkLimitsMatchTheRoversController)
{
  // IkController.h's constructor, verbatim.
  EXPECT_DOUBLE_EQ(kIkTimestepSeconds, 0.06);
  EXPECT_DOUBLE_EQ(kBetaDotMax, 1.5);
  EXPECT_DOUBLE_EQ(kBetaDdotMax, 250.0);
  EXPECT_DOUBLE_EQ(kAccelerationFactor, 3.0);
}
