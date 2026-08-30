#ifndef NAVI_SIM_IK__ASTEROPE_PARAMS_HPP_
#define NAVI_SIM_IK__ASTEROPE_PARAMS_HPP_

#include <array>

namespace navi_sim_ik
{

/// Asterope's chassis geometry and IK limits, as the rover itself holds them.
///
/// Transcribed from two files in the rover's own stack:
///   - the `#if ASTEROPE` block of `bemacontroller/src/RoverParameters.h`
///     (the eight wheel offsets), and
///   - the `IkController` constructor in
///     `bemacontroller/src/betterIK/IkController.h` (TS and the three limits).
/// The model itself is vendored in `../../vendor242/`; see its VENDOR242.md.
///
/// Unlike the 2.41 model in `../../vendor/`, model 2.42 has no geometry baked
/// into the generated code: it takes it at runtime on the root inport
/// `hParams[8]`. This header is where the simulation's copy of that geometry
/// lives, and the only place it lives.
///
/// FLOAT, NOT DOUBLE, ON PURPOSE. The rover declares these `const float` and
/// hands them to `IkController` in a `std::vector<float>`, which the
/// constructor widens into `hParams`. So the double the model actually sees is
/// a widened float. Writing `0.45527` as a double literal here would be a
/// different number in the 8th decimal place, and the simulation would no
/// longer be running the rover's arithmetic - which is the entire point of
/// SP4 (see docs/superpowers/specs/autonomy-plan.md §1.4).
inline constexpr float kWheel1X = 0.45527f;
inline constexpr float kWheel1Y = -0.44385f;
inline constexpr float kWheel2X = 0.45527f;
inline constexpr float kWheel2Y = 0.44385f;
inline constexpr float kWheel3X = -0.45527f;

/// NOTE, and left wrong on purpose: the rover has `wheel3y = 0.44285` where
/// wheels 1, 2 and 4 use `0.44385` - a 1 mm asymmetry in a chassis that is
/// otherwise perfectly symmetric, and almost certainly an upstream typo in
/// RoverParameters.h.
///
/// It is transcribed as the rover has it. Bit-identical arithmetic with the
/// machine that actually drives beats "fixed" arithmetic that no rover runs:
/// SP10's feasibility clamp is only trustworthy if the model it clamps
/// against is the model the wheels obey. Correcting it here would make the
/// simulation quietly disagree with the rover in exactly the regime the clamp
/// cares about.
///
/// If this is corrected upstream, correct it here in the same commit and
/// regenerate the golden table in `../../test/test_ik_parity_242.cpp`.
inline constexpr float kWheel3Y = 0.44285f;

inline constexpr float kWheel4X = -0.45527f;
inline constexpr float kWheel4Y = -0.44385f;

/// The order `IkController`'s constructor is called with, from
/// `bemacontroller/src/BemaServer.cpp:32`, copied element-by-element into
/// `ExtU_kinematics_T::hParams[0..7]`.
inline constexpr std::array<float, 8> kAsteropeHParams{
  {kWheel1X, kWheel1Y, kWheel2X, kWheel2Y, kWheel3X, kWheel3Y, kWheel4X, kWheel4Y}};

/// The IK's tick. The rover hardcodes 0.06 s in `IkController`'s constructor
/// and its update thread sleeps that long; the simulation ticks the same
/// period off `/clock`.
inline constexpr double kIkTimestepSeconds = 0.06;

/// Steering rate ceiling, rad/s. `IkController.h`.
inline constexpr double kBetaDotMax = 1.5;

/// Steering acceleration ceiling, rad/s^2. `IkController.h`.
inline constexpr double kBetaDdotMax = 250.0;

/// Drive acceleration, rad/s^2. `IkController.h`.
inline constexpr double kAccelerationFactor = 3.0;

}  // namespace navi_sim_ik

#endif  // NAVI_SIM_IK__ASTEROPE_PARAMS_HPP_
