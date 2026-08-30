# SP4: Re-vendor betterIK 2.42 (Asterope) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The simulation runs the rover's own inverse kinematics — Simulink model 2.42, grt target, Asterope geometry fed in through the `hParams` inport exactly as `bemacontroller` feeds it — so that `twist_shaper` (SP10) can clamp real commands against arithmetic the rover actually executes.

**Architecture:** The 2.42 sources are copied from the local `bemacontroller/` clone into a new, frozen `sim/src/navi_sim_ik/vendor242/` beside the existing (untouched) 2.41 `vendor/`, and compiled as their own static library `navi_sim_ik_model242`. A small hand-written header `include/navi_sim_ik/asterope_params.hpp` carries the eight wheel offsets and the three IK limits, transcribed bit-for-bit from `RoverParameters.h` and `IkController.h`, and `SimIkStepper` populates the model's `ExtU_kinematics_T` from it once in its constructor. A golden harness compiled from the same vendored sources generates a command→wheel-output table, which a table-driven gtest pins as literals.

**Tech Stack:** C++17, ROS 2 Humble, `ament_cmake`, `ament_cmake_gtest`, colcon. Simulink Coder 9.9 (R2023a) generated C++, grt.tlc target.

**Spec:** `docs/superpowers/specs/autonomy-plan.md` — §1.4 (the model swap and why), §8 (SP4 row: "Re-vendor `betterIK` (2.42, Asterope `hParams`) into `navi_sim_ik`; sim and rover run identical arithmetic", no dependencies, first sub-project), §9 rung 1 (pure functions on the laptop, including "`twist_shaper` clamping against the real 2.42 IK"), §10 (speeds: manual cap 0.05 m/s / 0.1 rad/s; autonomy stages 0.05 → 0.15 → 0.30 → 0.45 m/s with `wz ≤ 0.4 rad/s`).

## Global Constraints

- **`sim/src/navi_sim_ik/vendor/` is read-only.** Never edit anything under it. The 2.41 model stays on disk and stays compiling; **this plan does not delete it** — removal is a later cleanup decision, made once nothing references it and someone has ruled on the licence note in `vendor/VENDOR.md`.
- **`/home/ole/star/Navi/bemacontroller/` is a read-only reference copy** of the rover's stack (untracked in git). Never modify anything under it. It is the source the vendored files are copied *from*.
- New code goes in `sim/src/navi_sim_ik/vendor242/`, with its own `VENDOR242.md` provenance file recording: source path on the rover clone, model version 2.42, grt.tlc, R2023a, date copied, md5 of `kinematics.cpp`, and the `wheel3y` note.
- Nothing under `ground_station/` imports rclpy; `ground_station/models.py` imports neither Qt nor ROS. **This plan does not touch `ground_station/` at all.**
- **Never publish to `/manual_twist` in any test.** The smoke test in Task 6 remaps the node's subscription to `/sim_test_twist` and runs on throwaway `ROS_DOMAIN_ID=91`, never domain 0.
- The sim must keep building and testing green after every task. No task leaves the build red.
- Commit at the end of every task with an explicit `git add <paths>` (never `git add -A`), never push.
- **ODR rule:** `vendor/` and `vendor242/` both define `class kinematics`, with different member layouts and different `ExternalInputs`/`ExtU_kinematics_T` shapes. They live in two separate static libraries with two separate include directories, and **no single binary may link both.** `test_vendored_model` links only `navi_sim_ik_model` (2.41); everything else links only `navi_sim_ik_model242`.

### Exact values — Asterope, from `bemacontroller/src/RoverParameters.h`, `#if ASTEROPE` block

```
wheel1x =  0.45527   wheel1y = -0.44385
wheel2x =  0.45527   wheel2y =  0.44385
wheel3x = -0.45527   wheel3y =  0.44285   <-- 0.44285, not 0.44385
wheel4x = -0.45527   wheel4y = -0.44385

wheelDiameterInM = 0.2492   steerConstant = 93.345   driveConstant = 52.0
kickbackRatio = -0.31       maximumMotorAngle = 38160
crabTurnSensitivity = 90    pointTurnSensitivity = 40
```

Only the eight wheel offsets reach the IK model — `IkController`'s constructor takes
`{wheel1x, wheel1y, wheel2x, wheel2y, wheel3x, wheel3y, wheel4x, wheel4y}` and writes
them into `hParams[0..7]` in that order (`bemacontroller/src/BemaServer.cpp:32`). The
other constants belong to the motor/joint conversion (`motorToJoint`/`jointToMotor`),
which the simulation does not run, and are **not** vendored.

**`wheel3y` is 0.44285 where wheels 1, 2 and 4 use 0.44385.** This is a suspected
upstream typo — a 1 mm asymmetry in a chassis that is otherwise perfectly symmetric.
**It is vendored as the rover has it.** Bit-identical arithmetic with the machine that
actually drives beats "fixed" arithmetic that no rover runs; a clamp tuned against a
corrected model would be wrong about the real one. Flagged in a comment in
`asterope_params.hpp` and in `VENDOR242.md`, and pinned by a test so it cannot be
"tidied up" silently.

### Exact values — IK limits, from `bemacontroller/src/betterIK/IkController.h` constructor

```
TS                  = 0.06    // seconds
beta_dot_max        = 1.5     // rad/s
beta_ddot_max       = 250.0   // rad/s^2
acceleration_factor = 3.0     // drive acceleration, rad/s^2
```

### Unit and sign convention (settled — do not re-derive, and do not convert twice)

- `ExtU_kinematics_T::U` is the yaw rate in **rad/s**, in the ordinary ROS
  (counter-clockwise-positive) sense.
- The rover reaches that value through two negations that cancel:
  `bema_bridge.py:70` sends `w = -degrees(msg.angular.z)` over msgpack-RPC, and
  `BemaServer::drive()` (`BemaServer.cpp:178`) computes `m_driveComms.u = -M_PI*w/180.f`.
  So `U == angular.z` in rad/s.
- **Therefore `SimIkStepper::step(vx, vy, yaw_rate)` assigns `yaw_rate` straight into
  `in_.U` with no negation and no degree conversion.** The deg/s pair lives entirely on
  the `bema_bridge` → `BemaServer` path, outside the model. Applying it again here would
  spin the simulated rover backwards at 1/57th the rate.
- `VX_out` / `VY_out` are body-frame m/s, unconverted on both paths.

### 2.41 → 2.42 interface differences (verified by reading both headers)

| | 2.41 (`vendor/ert_rtw/kinematics.h`) | 2.42 (`vendor242/kinematics.h`) |
|---|---|---|
| input struct | `kinematics::ExternalInputs` (nested) | `ExtU_kinematics_T` (free) |
| output struct | `kinematics::ExternalOutputs` (nested) | `ExtY_kinematics_T` (free) |
| array members | `std::array<real_T, N>` | plain `real_T[N]` |
| yaw-rate field | `U_p` | `U` |
| geometry | baked in (Merope) | `real_T hParams[8]` inport |
| extra inports | — | `beta_ddot_max`, `beta_dot_max`, `acceleration_factor` |
| output fields | identical names and meanings: `Beta_dot[4]`, `input_ICR[2]`, `controller_ICR[2]`, `feasable_ICR[2]`, `omega[4]`, `current_ICR[2]`, `beta_next[4]`, `indirect_mode`, `border_ICR[2]`, `eta_dot_constrained[3]`, `eta_dot_ref_init[3]` | |

`initialize()` on 2.42 calls `rt_InitInfAndNaN` and seeds the zero-crossing state, so
`ModelPrevZCStateInit()` does **not** need calling separately (verified at
`kinematics.cpp:1550`).

### Design decisions taken here (rationale, one line each)

1. **hParams mechanism.** 2.42/grt exposes geometry as a **root inport `hParams[8]` inside `ExtU_kinematics_T`**, not as a Simulink instance-parameter (`InstP`) struct — there is no `InstP` in this generation — so the wrapper holds a persistent `ExtU_kinematics_T` with `hParams` filled once in its constructor and re-supplied on every `setExternalInputs()` (which copies the whole struct by value), exactly as `IkController` does with its `m_in`.
2. **Float widening is load-bearing.** The rover holds the offsets as `const float` and passes them through a `std::vector<float>`, so the model sees a *widened float*, not the decimal literal — `asterope_params.hpp` declares them `float` and widens at assignment for the same reason.
3. **`IkController.{h,cpp}` are not vendored.** It is a `std::thread` that sleeps 0.06 s of wall time in a loop; the simulation already has its own tick, driven by `/clock`, and a second wall-clock ticker inside it would desync silently — the sim's `SimIkStepper` is that wrapper's replacement, not its client.
4. **`.mat` / `.dmr` metadata is not vendored.** `buildInfo.mat`, `codeInfo.mat`, `compileInfo.mat`, `rtwtypeschksum.mat`, `codedescriptor.dmr` (0.74 MB) and `rtw_proj.tmw` are MATLAB build metadata: nothing compiles them, nothing in this repo can read them, and the provenance they carry is recorded in prose in `VENDOR242.md` where a human can actually check it.
5. **`rtmodel.h` is not vendored.** It is the grt harness shim and only `#include`s `kinematics.h`; nothing in this package builds a grt main.
6. **`test/test_vendored_model.cpp` (2.41) keeps building against `vendor/`.** It costs one static library and no runtime, and while 2.41 stays on disk it is the only thing proving that frozen copy is still complete — deleting the test while keeping the code leaves an unguarded directory that will rot unnoticed. It is renamed in no way and its assertions are untouched; the ODR rule above keeps it away from 2.42.
7. **Golden values come from a harness, not from this plan.** `test/golden_harness_242.cpp` is committed (so the table can be regenerated) but is deliberately *not* a CMake target — it is compiled ad hoc with a `g++` line recorded in its own header comment, so it can never affect the package build.

---

### Task 1: Vendor betterIK 2.42 into `vendor242/`, with provenance, and build it

**Files:**
- Create `sim/src/navi_sim_ik/vendor242/` — 18 files copied verbatim (list in Step 3)
- Create `sim/src/navi_sim_ik/vendor242/VENDOR242.md`
- Modify `sim/src/navi_sim_ik/CMakeLists.txt`
- Test: create `sim/src/navi_sim_ik/test/test_vendored_model_242.cpp`

**Interfaces:**
- *Consumes:* `/home/ole/star/Navi/bemacontroller/src/betterIK/*` (read-only reference).
- *Produces:* CMake target `navi_sim_ik_model242` (STATIC), exporting include dir `vendor242/`; C++ symbols `class kinematics` (`initialize()`, `step()`, `setExternalInputs(const ExtU_kinematics_T *)`, `getExternalOutputs() -> const ExtY_kinematics_T &`), structs `ExtU_kinematics_T`, `ExtY_kinematics_T`, typedef `real_T` (`double`), `boolean_T`.
- *Produces:* gtest binary `test_vendored_model_242`.

**Steps:**

- [ ] **Step 1: Write the failing smoke test.** Create `sim/src/navi_sim_ik/test/test_vendored_model_242.cpp`:

```cpp
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
```

- [ ] **Step 2: Run it and watch it fail to build.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik'
  ```
  Expect: the build succeeds and simply does not build the new test (it is not in `CMakeLists.txt` yet), so also confirm the file is unbuilt with
  ```
  ls /home/ole/star/Navi/sim/build/navi_sim_ik/test_vendored_model_242
  ```
  Expect: `ls: cannot access ... No such file or directory`.

- [ ] **Step 3: Copy the 2.42 sources.** Run exactly:
  ```
  mkdir -p /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242
  cp /home/ole/star/Navi/bemacontroller/src/betterIK/kinematics.cpp \
     /home/ole/star/Navi/bemacontroller/src/betterIK/kinematics_data.cpp \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rt_nonfinite.cpp \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtGetInf.cpp \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtGetNaN.cpp \
     /home/ole/star/Navi/bemacontroller/src/betterIK/kinematics.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/kinematics_private.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/kinematics_types.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rt_defines.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rt_nonfinite.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtGetInf.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtGetNaN.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtw_continuous.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtw_solver.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/rtwtypes.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/tmwtypes.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/multiword_types.h \
     /home/ole/star/Navi/bemacontroller/src/betterIK/zero_crossing_types.h \
     /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242/
  ```
  Then verify the copy is faithful and complete:
  ```
  ls -1 /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242 | wc -l
  md5sum /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242/kinematics.cpp
  ```
  Expect: `18`, and `59e45d5d1bf4a9dd4e3197aab4c05822`. If the md5 differs, **stop** — the reference clone has moved and the provenance in Step 4 would be a lie.

- [ ] **Step 4: Write the provenance file.** Create `sim/src/navi_sim_ik/vendor242/VENDOR242.md`:

```markdown
# Vendored kinematics — model 2.42, Asterope

Copied verbatim from the bema controller. **Do not edit these files**: they are
Simulink-generated and are replaced by regenerating the model upstream. This
directory is frozen the same way `../vendor/` is.

- Origin: `ssh://git@gitlab.star-dresden.de:11022/star-projekte/merope/bema/bemacontroller.git`
- Commit: `bfeae1575b64563661b8da06b486fd99f4d5603f`
- Local clone copied from: `/home/ole/star/Navi/bemacontroller/src/betterIK/`
- Copied: 2026-08-30
- Model version: **2.42**
- Target: **grt.tlc**, Simulink Coder 9.9 (**R2023a**), generated 2023-09-06
- `md5sum kinematics.cpp` = `59e45d5d1bf4a9dd4e3197aab4c05822`
- `md5sum kinematics.h` = `a389f45ddacfbdd751d78fa89a4325ea`
- `md5sum kinematics_data.cpp` = `98f68c531bb5787cfb622299b8f544cf`

## Why a second vendor directory instead of replacing `../vendor/`

`../vendor/` holds model **2.41** (ert.tlc, R2022a) with **Merope** geometry
baked into the generated code. It is frozen by project rule and stays where it
is, unreferenced by anything but its own regression test, until someone decides
to remove it. This directory holds the model the rover actually runs.

The reason for vendoring at all, unchanged from `../vendor/VENDOR.md`: the
simulation must run the same arithmetic as the rover, so a disagreement between
them is a real disagreement and not a porting artifact. 2.41 could not deliver
that — different model version, different geometry — which is what
`docs/superpowers/specs/autonomy-plan.md` §1.4 records.

Copied rather than referenced across the working tree because
`bemacontroller/` is an untracked nested repository: a clone of this project
would not have it, and the build must not depend on a directory that may not
exist.

## What was copied, and what was not

Copied (18 files): the five translation units the rover's own
`src/betterIK/CMakeLists.txt` compiles — `kinematics.cpp`,
`kinematics_data.cpp`, `rt_nonfinite.cpp`, `rtGetInf.cpp`, `rtGetNaN.cpp` —
plus every header they transitively include: `kinematics.h`,
`kinematics_private.h`, `kinematics_types.h`, `rt_defines.h`,
`rt_nonfinite.h`, `rtGetInf.h`, `rtGetNaN.h`, `rtw_continuous.h`,
`rtw_solver.h`, `rtwtypes.h`, `tmwtypes.h`, `multiword_types.h`,
`zero_crossing_types.h`.

Not copied, and why:

- `IkController.{h,cpp}` — the rover's wrapper is a `std::thread` sleeping
  0.06 s of wall time in a loop. The simulation already has its own tick,
  driven by `/clock`; a second, wall-clock ticker inside it would drift out of
  step silently. `navi_sim_ik`'s `SimIkStepper` is that wrapper's replacement.
- `rtmodel.h` — the grt harness shim; it only `#include`s `kinematics.h`, and
  nothing here builds a grt main.
- `buildInfo.mat`, `codeInfo.mat`, `compileInfo.mat`, `rtwtypeschksum.mat`,
  `codedescriptor.dmr` (0.74 MB), `rtw_proj.tmw` — MATLAB build metadata.
  Nothing compiles them and nothing in this repository can read them; the
  provenance they carry is written out above in a form a human can check.
- `CMakeLists.txt` — the rover's, describing the rover's `ik_lib` target.

## Geometry is not in these files

Unlike 2.41, this model takes the chassis geometry at runtime through the root
inport `hParams[8]`, ordered
`{wheel1x, wheel1y, wheel2x, wheel2y, wheel3x, wheel3y, wheel4x, wheel4y}`.
The Asterope values live in `../include/navi_sim_ik/asterope_params.hpp`,
transcribed from `bemacontroller/src/RoverParameters.h` (`#if ASTEROPE`).

**Note on `wheel3y`.** The rover's `RoverParameters.h` has
`wheel3y = 0.44285` while wheels 1, 2 and 4 use `0.44385` — a 1 mm asymmetry
in an otherwise perfectly symmetric chassis, and almost certainly an upstream
typo. It is transcribed **as the rover has it**. Bit-identical arithmetic with
the machine that actually drives beats "fixed" arithmetic no rover runs: a
feasibility clamp tuned against a corrected model would be wrong about the
real one. If this is ever corrected upstream, correct it here in the same
commit and regenerate the golden table in `../test/test_ik_parity_242.cpp`.

## Licence

Fifteen of the eighteen files open with this banner, copied verbatim from the
generated sources:

```
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
```

Files carrying it: `kinematics.cpp`, `kinematics.h`, `kinematics_data.cpp`,
`kinematics_private.h`, `kinematics_types.h`, `rt_defines.h`,
`rt_nonfinite.cpp`, `rt_nonfinite.h`, `rtGetInf.cpp`, `rtGetInf.h`,
`rtGetNaN.cpp`, `rtGetNaN.h`, `rtwtypes.h`, `multiword_types.h`,
`zero_crossing_types.h`.

Files that do NOT carry it: `rtw_continuous.h`, `rtw_solver.h`, `tmwtypes.h`.

As with `../vendor/VENDOR.md`: this was noted at the time of vendoring and has
**not** been assessed by anyone with the authority to rule on whether it
affects this project's use of the code.
```

- [ ] **Step 5: Wire the library and the test into CMake.** In `sim/src/navi_sim_ik/CMakeLists.txt`, insert after the `target_compile_options(navi_sim_ik_model PRIVATE -w)` line (i.e. after the 2.41 block, before `add_library(navi_sim_ik_stepper ...)`):

```cmake
# The rover's model: 2.42, grt.tlc, R2023a, geometry supplied at runtime on
# the hParams inport. Simulink-generated, compiled as found; its warnings are
# not ours to fix and would drown anything we do need to see.
#
# Listed explicitly rather than GLOBed: a GLOB would silently absorb any file
# dropped into a directory that is supposed to be frozen.
#
# CRITICAL: this library and navi_sim_ik_model above both define
# `class kinematics`, with different layouts and different input structs. No
# single binary may link both. test_vendored_model links only 2.41; everything
# else links only this.
add_library(navi_sim_ik_model242 STATIC
  vendor242/kinematics.cpp
  vendor242/kinematics_data.cpp
  vendor242/rt_nonfinite.cpp
  vendor242/rtGetInf.cpp
  vendor242/rtGetNaN.cpp)
target_include_directories(navi_sim_ik_model242 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor242>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_options(navi_sim_ik_model242 PRIVATE -w)
```

  and inside the `if(BUILD_TESTING)` block, after the existing
  `target_link_libraries(test_vendored_model navi_sim_ik_model)` line:

```cmake
  ament_add_gtest(test_vendored_model_242 test/test_vendored_model_242.cpp)
  target_link_libraries(test_vendored_model_242 navi_sim_ik_model242)
```

- [ ] **Step 6: Build and run the test.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
  ```
  Expect: PASS. `test_vendored_model_242` reports 3 tests passed; `test_vendored_model`, `test_sim_ik_stepper` and `test_external_pose` all still pass unchanged.

- [ ] **Step 7: Commit.**
  ```
  cd /home/ole/star/Navi && git add sim/src/navi_sim_ik/vendor242 sim/src/navi_sim_ik/test/test_vendored_model_242.cpp sim/src/navi_sim_ik/CMakeLists.txt && git commit -m "Vendor the rover's betterIK 2.42 alongside the frozen 2.41"
  ```

---

### Task 2: The Asterope parameter header

**Files:**
- Create `sim/src/navi_sim_ik/include/navi_sim_ik/asterope_params.hpp`
- Test: create `sim/src/navi_sim_ik/test/test_asterope_params.cpp`
- Modify `sim/src/navi_sim_ik/CMakeLists.txt` (register the new gtest)
- Modify `sim/src/navi_sim_ik/test/test_vendored_model_242.cpp` (use the header instead of its inline literals)

**Interfaces:**
- *Consumes:* nothing at compile time (header-only, `<array>` only). Transcribed from `bemacontroller/src/RoverParameters.h` and `bemacontroller/src/betterIK/IkController.h`.
- *Produces:* namespace `navi_sim_ik`: `constexpr float kWheel1X, kWheel1Y, kWheel2X, kWheel2Y, kWheel3X, kWheel3Y, kWheel4X, kWheel4Y`; `constexpr std::array<float, 8> kAsteropeHParams`; `constexpr double kIkTimestepSeconds`, `kBetaDotMax`, `kBetaDdotMax`, `kAccelerationFactor`.

**Steps:**

- [ ] **Step 1: Write the failing test.** Create `sim/src/navi_sim_ik/test/test_asterope_params.cpp`:

```cpp
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
```

- [ ] **Step 2: Run it and watch it fail.** First register it in `sim/src/navi_sim_ik/CMakeLists.txt`, inside `if(BUILD_TESTING)`, after the `test_vendored_model_242` lines:

```cmake
  ament_add_gtest(test_asterope_params test/test_asterope_params.cpp)
  target_include_directories(test_asterope_params PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
```

  then run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik'
  ```
  Expect failure: `fatal error: navi_sim_ik/asterope_params.hpp: No such file or directory`.

- [ ] **Step 3: Write the header.** Create `sim/src/navi_sim_ik/include/navi_sim_ik/asterope_params.hpp`:

```cpp
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
```

- [ ] **Step 4: Run the test.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
  ```
  Expect: PASS, `test_asterope_params` 5 tests passed, everything else still green.

- [ ] **Step 5: Remove the duplicated literals from Task 1's smoke test.** In `sim/src/navi_sim_ik/test/test_vendored_model_242.cpp`, replace the `set_asterope_geometry` and `set_ik_limits` helper bodies (and delete the comment paragraph about them being inline "only for this first task") with:

```cpp
#include "navi_sim_ik/asterope_params.hpp"

namespace
{
void set_asterope_geometry(ExtU_kinematics_T & in)
{
  for (int i = 0; i < 8; ++i) {
    in.hParams[i] = navi_sim_ik::kAsteropeHParams[i];
  }
}

void set_ik_limits(ExtU_kinematics_T & in)
{
  in.TS = navi_sim_ik::kIkTimestepSeconds;
  in.beta_dot_max = navi_sim_ik::kBetaDotMax;
  in.beta_ddot_max = navi_sim_ik::kBetaDdotMax;
  in.acceleration_factor = navi_sim_ik::kAccelerationFactor;
}
```

  Leave `GeometryOnHParamsActuallyReachesTheArithmetic` alone: its two inline
  arrays are the *point* of that test (it compares Asterope against Merope) and
  must not be routed through the header.

- [ ] **Step 6: Re-run and confirm still green.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
  ```
  Expect: PASS, all four test binaries green.

- [ ] **Step 7: Commit.**
  ```
  cd /home/ole/star/Navi && git add sim/src/navi_sim_ik/include/navi_sim_ik/asterope_params.hpp sim/src/navi_sim_ik/test/test_asterope_params.cpp sim/src/navi_sim_ik/test/test_vendored_model_242.cpp sim/src/navi_sim_ik/CMakeLists.txt && git commit -m "Transcribe Asterope's geometry and IK limits, typo and all"
  ```

---

### Task 3: The golden harness

**Files:**
- Create `sim/src/navi_sim_ik/test/golden_harness_242.cpp`
- Create `/tmp/sp4_golden_table.txt` (transient output — not committed)

**Interfaces:**
- *Consumes:* `vendor242/kinematics.h` (`kinematics`, `ExtU_kinematics_T`, `ExtY_kinematics_T`), `navi_sim_ik/asterope_params.hpp` (`kAsteropeHParams`, `kIkTimestepSeconds`, `kBetaDotMax`, `kBetaDdotMax`, `kAccelerationFactor`).
- *Produces:* an executable at `/tmp/golden_harness_242` printing 10 C++ brace-initialiser rows on stdout, and the file `/tmp/sp4_golden_table.txt` holding them, consumed verbatim by Task 4.

**Why it is not a CMake target:** it is a code generator, not a test. Keeping it out of the build means it can never break the package, never install, and never accidentally link the 2.41 library; the `g++` line that builds it is recorded in the file's own header comment so anyone can reproduce the table.

**Steps:**

- [ ] **Step 1: Write the harness.** Create `sim/src/navi_sim_ik/test/golden_harness_242.cpp`:

```cpp
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
```

- [ ] **Step 2: Build and run it, capturing the table.** Run exactly:
  ```
  g++ -std=c++17 -O2 -w \
    -I/home/ole/star/Navi/sim/src/navi_sim_ik/vendor242 \
    -I/home/ole/star/Navi/sim/src/navi_sim_ik/include \
    /home/ole/star/Navi/sim/src/navi_sim_ik/test/golden_harness_242.cpp \
    /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242/*.cpp \
    -o /tmp/golden_harness_242 && /tmp/golden_harness_242 | tee /tmp/sp4_golden_table.txt
  ```
  Expect: 40 lines of output (4 lines per command × 10 commands), each row of
  the form `  {{0.05, 0, 0},` then three brace lines. Then check for the
  failure that would poison every downstream number:
  ```
  grep -ci 'nan\|inf' /tmp/sp4_golden_table.txt
  ```
  Expect: `0`. If it is not 0, **stop** — a non-finite output means the model
  never initialised its non-finites, and the vendored copy is incomplete.

- [ ] **Step 3: Sanity-check two rows by hand before trusting the table.**
  Confirm the zero-command row's `omega` and `eta_dot_constrained` are all
  exactly `0`, and that the `{0.45, 0, 0.4}` row's `eta_dot_constrained[2]` is
  within 0.05 of `0.4`:
  ```
  head -4 /tmp/sp4_golden_table.txt
  sed -n '29,32p' /tmp/sp4_golden_table.txt
  ```
  Expect: the first row's second and third brace lines are all zeros; the
  eighth row's last brace line ends with a yaw rate near 0.4. If the zero
  command produces motion, **stop** — nothing downstream is trustworthy.
  (The zero command's `beta_next` is *not* zero: the model parks the wheels in
  its point-turn configuration when nothing is asked of it. That is real model
  behaviour and is pinned as-is.)

- [ ] **Step 4: Commit the generator.**
  ```
  cd /home/ole/star/Navi && git add sim/src/navi_sim_ik/test/golden_harness_242.cpp && git commit -m "Add the generator for the 2.42 golden parity table"
  ```

---

### Task 4: The numeric-parity test

**Files:**
- Create `sim/src/navi_sim_ik/test/test_ik_parity_242.cpp`
- Modify `sim/src/navi_sim_ik/CMakeLists.txt`

**Interfaces:**
- *Consumes:* `/tmp/sp4_golden_table.txt` (Task 3), `navi_sim_ik/asterope_params.hpp` (Task 2), the `navi_sim_ik_model242` target (Task 1).
- *Produces:* gtest binary `test_ik_parity_242` — 10 parameterised cases, each asserting 11 doubles.

**Steps:**

- [ ] **Step 1: Write the test with the table left empty, so it fails.** Create `sim/src/navi_sim_ik/test/test_ik_parity_242.cpp`:

```cpp
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
  // <<< TABLE GOES HERE >>>
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
```

- [ ] **Step 2: Register it and run it, failing.** In
  `sim/src/navi_sim_ik/CMakeLists.txt`, inside `if(BUILD_TESTING)` after the
  `test_asterope_params` lines:

```cmake
  ament_add_gtest(test_ik_parity_242 test/test_ik_parity_242.cpp)
  target_link_libraries(test_ik_parity_242 navi_sim_ik_model242)
```

  then run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik'
  ```
  Expect the test to fail. Depending on the compiler's zero-size-array
  diagnostic this is either a build error on `kGolden` (g++ under
  `-pedantic-errors` reports `error: zero-size array 'kGolden'`) or, more
  likely, a successful build whose `test_ik_parity_242` fails immediately on
  "the golden table is empty" — ament's default flags do not enable
  `-pedantic-errors`, so a zero-size array is accepted as an extension and the
  runtime `ASSERT_GT(sizeof(kGolden) / sizeof(kGolden[0]), 0u)` guard in the
  test body is what actually catches it. Either is the intended red.

- [ ] **Step 3: Paste the generated table in.** Replace the line
  `  // <<< TABLE GOES HERE >>>` in `test_ik_parity_242.cpp` with the exact
  contents of `/tmp/sp4_golden_table.txt` (all 40 lines, unedited — do not
  round, do not reformat, do not retype). If `/tmp/sp4_golden_table.txt` is
  missing, regenerate it with Task 3's Step 2 command before continuing.
  Verify the paste landed:
  ```
  grep -c '^   {' /home/ole/star/Navi/sim/src/navi_sim_ik/test/test_ik_parity_242.cpp
  ```
  Expect: `30` (three brace lines per case × 10 cases).

- [ ] **Step 4: Run the test.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
  ```
  Expect: PASS, `test_ik_parity_242` 2 tests passed, and all other test
  binaries still green.

- [ ] **Step 5: Prove the test can actually fail.** Temporarily change
  `kWheel3Y` in `include/navi_sim_ik/asterope_params.hpp` from `0.44285f` to
  `0.44385f` (the "corrected" value), rebuild and re-run just the parity test:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && /home/ole/star/Navi/sim/build/navi_sim_ik/test_ik_parity_242'
  ```
  Expect: FAIL, with `EveryCommandReproducesTheRoversArithmetic` reporting
  differences on several rows. This is the whole point of the test — a 1 mm
  geometry change is visible. Then **revert the header back to `0.44285f`**,
  rebuild, and confirm PASS again with the same command. Do not commit while
  the header is modified.

- [ ] **Step 6: Commit.**
  ```
  cd /home/ole/star/Navi && git add sim/src/navi_sim_ik/test/test_ik_parity_242.cpp sim/src/navi_sim_ik/CMakeLists.txt && git commit -m "Pin the 2.42 Asterope wheel outputs against a generated golden table"
  ```

---

### Task 5: Switch `SimIkStepper` to the 2.42 model

**Files:**
- Modify `sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp`
- Modify `sim/src/navi_sim_ik/src/sim_ik_stepper.cpp`
- Modify `sim/src/navi_sim_ik/CMakeLists.txt`
- Test: modify `sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp` (re-measured baselines only)

**Interfaces:**
- *Consumes:* `navi_sim_ik_model242` (Task 1), `navi_sim_ik/asterope_params.hpp` (Task 2).
- *Produces:* unchanged public surface — `navi_sim_ik::SimIkStepper` with `SimIkStepper(double ts = kIkTimestepSeconds)`, `step(double vx, double vy, double yaw_rate)`, `targets() -> const WheelTargets &`, `pose() -> const Pose2D &`, `set_pose(const Pose2D &)`, `indirect_mode() -> bool`, `feasible_icr() -> std::array<double,2>`, `achieved_velocity() -> const Velocity2D &`; `navi_sim_ik::WHEEL_CORNERS`, `Pose2D`, `WheelTargets`, `Velocity2D`. `sim_ik_node.cpp` compiles unchanged against it.

**Steps:**

- [ ] **Step 1: Repoint the header at 2.42.** In
  `sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp`:

  a. After `#include <array>`, add:
```cpp
#include "kinematics.h"
#include "navi_sim_ik/asterope_params.hpp"
```
  (replacing the existing bare `#include "kinematics.h"` line — there must be
  exactly one, and it now resolves to `vendor242/kinematics.h`.)

  b. Change the constructor declaration from
  `explicit SimIkStepper(double ts = 0.06);` to:
```cpp
  /// `ts` defaults to the rover's own IK period; see asterope_params.hpp.
  explicit SimIkStepper(double ts = kIkTimestepSeconds);
```

  c. Change the private member from
  `kinematics::ExternalInputs in_{};` to:
```cpp
  // Model 2.42 takes the chassis geometry at runtime on hParams, so this
  // struct is persistent and carries it: setExternalInputs() copies the whole
  // struct by value on every tick, exactly as the rover's IkController does
  // with its own m_in. Filled once in the constructor.
  ExtU_kinematics_T in_{};
```

  d. Extend the class doc comment above `class SimIkStepper` with:
```
/// The model is the rover's own: Simulink 2.42, grt target, R2023a, with
/// Asterope's geometry fed in through hParams (../../vendor242/VENDOR242.md).
/// The simulation runs the arithmetic the wheels obey, which is what makes a
/// disagreement between sim and rover a real disagreement — and what makes
/// SP10's feasibility clamp trustworthy.
```

- [ ] **Step 2: Rewrite the stepper's model glue.** Replace the constructor and
  the first half of `step()` in `sim/src/navi_sim_ik/src/sim_ik_stepper.cpp`.
  The complete new file, from the top through the `feasible_icr_` assignment
  (everything below it — the achieved-velocity comment block and the pose
  integration — stays exactly as it is):

```cpp
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
```

  (Keep the rest of `step()` and the closing `}  // namespace navi_sim_ik`
  byte-for-byte as they are.)

- [ ] **Step 3: Relink the stepper library.** In
  `sim/src/navi_sim_ik/CMakeLists.txt`, change
  `target_link_libraries(navi_sim_ik_stepper navi_sim_ik_model)` to:
```cmake
# 2.42, not 2.41: this is the library the node and every downstream consumer
# get. navi_sim_ik_model (2.41) is now reachable only from
# test_vendored_model - see the ODR note above.
target_link_libraries(navi_sim_ik_stepper navi_sim_ik_model242)
```

- [ ] **Step 4: Build and run the tests — expect two stepper tests to fail.**
  Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && /home/ole/star/Navi/sim/build/navi_sim_ik/test_sim_ik_stepper'
  ```
  Expect: the build succeeds and **two** tests fail, because several of
  `test_sim_ik_stepper.cpp`'s expectations are explicitly documented *in the
  test file itself* as measured baselines of the old (2.41, Merope) model, and
  the geometry has changed:
  - `SimIkStepper.TurningInPlaceChangesYawWithoutTravelling` — the `x`
    baseline. Under Merope the spin-up transient went one way; under Asterope
    it goes the other, and this is the one sign change in the whole switch.
  - `SimIkStepper.YawIsIntegratedFromTheStartOfStepNotTheEnd` — the `yaw`
    literal (the `x` and `y` literals happen to stay inside their ±0.02 bands).

  Everything else — the structural assertions about direction, magnitude and
  frame — should pass untouched, which is the evidence that this is a change of
  geometry and not a change of behaviour. **If a third test fails, stop and
  investigate before editing any literal:** the remaining tests are structural
  (`> 1.0`, `< 1.5`, exact zeros, body-frame `vx ≈ 0.5`), and a structural
  failure means the wiring is wrong, not the numbers.

  For calibration, these values were measured during planning from a scratch
  build of the same 2.42 sources with the same Asterope parameters, replicating
  the stepper's integration exactly. Step 5 re-measures from the real test
  output and uses what it observes; these are what to expect, and a number
  wildly away from them means something else changed:
  `TurningInPlaceChangesYawWithoutTravelling` → `x ≈ 0.1787`, `y ≈ 0.0190`;
  `YawIsIntegratedFromTheStartOfStepNotTheEnd` → `x ≈ -0.0929653`,
  `y ≈ 1.6378290`, `yaw ≈ 3.3671992`; and for the tests that still pass,
  `DrivingForwardMovesAlongXAndNotAcross` → `y ≈ 0.0813`, `yaw ≈ 0.0581`.

- [ ] **Step 5: Re-measure and update only the measured baselines.** Read the
  actual values out of the gtest failure output (gtest prints
  "The difference between stepper.pose().x and 0.1787 is ...", from which the
  observed value is recoverable; if it is easier, add a temporary
  `std::printf` — and remove it before committing). Then, in
  `sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp`:

  a. In `TurningInPlaceChangesYawWithoutTravelling`, replace
  `EXPECT_NEAR(stepper.pose().x, -0.0792, 0.03);` with the observed value at
  the same ±0.03 tolerance, and replace the comment paragraph above it with:
```cpp
  // The IK measurably produces some transient x travel while spinning up
  // from a standing start (the wheels take a moment to swing into the
  // turn-in-place configuration, and that transient is real motion the
  // vehicle would actually make). Measured baseline from the 2.42 model with
  // Asterope geometry, not analytically derived; the +/-0.03 band around it
  // is a two-sided regression guard rather than a ceiling, so a change that
  // doubled the real transient would also fail. Note the sign: under the old
  // 2.41/Merope model this transient went the other way, which is a change of
  // chassis geometry and not of behaviour.
```

  b. In `YawIsIntegratedFromTheStartOfStepNotTheEnd`, replace all three
  literals (`-0.0841218`, `1.6467001`, `3.2995396`) with the observed values at
  the same ±0.02 tolerance, and append to its comment block:
```cpp
  // Re-measured for the 2.42 model with Asterope geometry (SP4). The
  // ordering bug this test exists to catch still shifts the endpoint by
  // roughly one step's yaw increment applied to the whole path - on the
  // order of 0.1 m, an order of magnitude past the tolerance here.
```

  c. In `DrivingForwardMovesAlongXAndNotAcross`, the existing `0.0764` and
  `0.0562` literals still pass within ±0.03, but they now name Merope numbers.
  Replace them with the observed Asterope values (same ±0.03) and change
  "the vendored IK" phrasing in its comment to
  "the 2.42 model with Asterope geometry". This test still passes, so there is
  no failure output to re-measure from — either add the temporary
  `std::printf` mentioned in Step 5, or use the Step 4 calibration values
  (y ≈ 0.08134, yaw ≈ 0.05806), which were measured from the same sources.

  d. In `DrivingForwardMovesAlongXAndNotAcross`'s comment, change the phrase
  `VY_out/U_p are zero` to `VY_out/U are zero` — `U_p` is the 2.41 field name
  and no longer exists.

  **Do not touch** any structural assertion: `EXPECT_GT(stepper.pose().x, 1.0)`,
  `EXPECT_GT(std::abs(stepper.pose().yaw), 0.5)`, `EXPECT_GT(pose().y, 1.0)`,
  `EXPECT_LT(std::abs(x - x_before), 1.5)`, the exact-zero drift assertions,
  `EXPECT_NEAR(achieved_velocity().vx, 0.5, 0.05)`, the `WHEEL_CORNERS`
  pinning, or the `set_pose` tests. Widening a tolerance to make a test pass is
  forbidden here — if a structural bound fails, the wiring is wrong.

- [ ] **Step 6: Run the full package.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build --packages-select navi_sim_ik && colcon test --packages-select navi_sim_ik && colcon test-result --verbose'
  ```
  Expect: PASS. All five binaries green — `test_vendored_model` (2.41, still
  building against the frozen `vendor/`), `test_vendored_model_242`,
  `test_asterope_params`, `test_ik_parity_242`, `test_sim_ik_stepper`, plus
  `test_external_pose`.

- [ ] **Step 7: Confirm no binary links both models.** Run:
  ```
  grep -n "navi_sim_ik_model\b" /home/ole/star/Navi/sim/src/navi_sim_ik/CMakeLists.txt
  ```
  Expect exactly four hits: `add_library` (line 19),
  `target_include_directories` (20), `target_compile_options` (24), and
  `target_link_libraries(test_vendored_model ...)` (47).
  If `navi_sim_ik_model` appears on any other target's link line, the ODR rule
  is broken — fix it before committing.

- [ ] **Step 8: Commit.**
  ```
  cd /home/ole/star/Navi && git add sim/src/navi_sim_ik/include/navi_sim_ik/sim_ik_stepper.hpp sim/src/navi_sim_ik/src/sim_ik_stepper.cpp sim/src/navi_sim_ik/test/test_sim_ik_stepper.cpp sim/src/navi_sim_ik/CMakeLists.txt && git commit -m "Run the simulation on the rover's 2.42 IK with Asterope geometry"
  ```

---

### Task 6: Whole-sim smoke test and provenance note

**Files:**
- Modify `sim/src/navi_sim_ik/vendor/VENDOR.md` (mark 2.41 superseded and unreferenced)
- Modify `PROJECT_SUMMARY.md` (the sim section's description of the vendored IK)
- Test: none created — this task runs the whole sim's existing tests plus a live-node check

**Interfaces:**
- *Consumes:* everything built in Tasks 1–5; the `sim_ik_node` executable; topic `/sim_odom` (`nav_msgs/msg/Odometry`), `/sim_cmd_vel` (`geometry_msgs/msg/Twist`), and the node's `/manual_twist` subscription, remapped.
- *Produces:* no new code. Evidence that the model swap did not break the simulation.

**Steps:**

- [ ] **Step 1: Build and test every sim package.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build && colcon test && colcon test-result --verbose'
  ```
  Expect: PASS across `navi_sim_ik`, `navi_sim_bringup` and `navi_sim_video`.
  If `navi_sim_bringup` or `navi_sim_video` fail, check whether they were
  already failing before this plan (`git stash` is not needed — those packages
  are untouched by Tasks 1–5); report it rather than "fixing" it here.

- [ ] **Step 2: Run the node headless on a throwaway domain.** The node is only
  put on the simulation clock by the launch file (`use_sim_time: True` in
  `sim.launch.py`), so run bare it ticks off the wall clock at 0.06 s and needs
  no Gazebo. Run, in one shell:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && source /home/ole/star/Navi/sim/install/setup.bash && export ROS_DOMAIN_ID=91 && ros2 run navi_sim_ik sim_ik_node --ros-args -r /manual_twist:=/sim_test_twist' &
  ```
  In a second shell (the publisher line waits 3 seconds first, for the node
  to come up):
  ```
  bash -c 'sleep 3 && source /opt/ros/humble/setup.bash && source /home/ole/star/Navi/sim/install/setup.bash && export ROS_DOMAIN_ID=91 && timeout 8 ros2 topic pub -r 20 /sim_test_twist geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"' &
  ```
  and, 2 seconds after that:
  ```
  bash -c 'sleep 2 && source /opt/ros/humble/setup.bash && source /home/ole/star/Navi/sim/install/setup.bash && export ROS_DOMAIN_ID=91 && timeout 5 ros2 topic echo /sim_odom --once'
  ```
  Expect: an `Odometry` message with `pose.pose.position.x` greater than zero
  and growing between repeated calls, and `child_frame_id: base_footprint`.
  `/sim_odom.twist` is deliberately left unpopulated (see `publish_motion()`),
  so check the velocity on `/sim_cmd_vel` instead:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=91 && timeout 5 ros2 topic echo /sim_cmd_vel --once'
  ```
  Expect `linear.x` near `0.0495`. **`/manual_twist` is never published to** —
  the remapping is what makes that true, so confirm the topic list shows
  `/sim_test_twist` and not `/manual_twist`:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=91 && ros2 topic list'
  ```
  Then stop the node with `pkill -x sim_ik_node` — `-x` only, never `pkill -f`
  with a pattern that would match the shell running it. The `timeout`-wrapped
  publisher terminates on its own; `pkill -x ros2` would not touch it anyway,
  since `ros2` is a Python console script and runs as `python3`, not `ros2`.

- [ ] **Step 3: Check the yaw sign end-to-end.** Restart the node exactly as in
  Step 2, then publish a positive `angular.z` and confirm the simulated rover
  turns counter-clockwise — the check that the deg/s negation pair was not
  applied a second time:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && source /home/ole/star/Navi/sim/install/setup.bash && export ROS_DOMAIN_ID=91 && timeout 6 ros2 topic pub -r 20 /sim_test_twist geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"' &
  ```
  then:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && source /home/ole/star/Navi/sim/install/setup.bash && export ROS_DOMAIN_ID=91 && timeout 5 ros2 topic echo /sim_odom --once'
  ```
  and, on `/sim_cmd_vel`:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=91 && timeout 5 ros2 topic echo /sim_cmd_vel --once'
  ```
  Expect: `pose.pose.orientation.z` **positive** and growing (a positive yaw),
  and, on `/sim_cmd_vel`, `angular.z` near `+0.1`. A negative `orientation.z`
  would mean the sign convention was inverted somewhere and Task 5 Step 2's
  comment was not honoured. Stop the processes as in Step 2.

- [ ] **Step 4: Mark the 2.41 vendor superseded.** Append to
  `sim/src/navi_sim_ik/vendor/VENDOR.md`:

```markdown
## Superseded, 2026-08-30

This model — 2.41, ert.tlc, R2022a, with **Merope** geometry baked into the
generated code — is **no longer what the simulation runs**. SP4 re-vendored the
rover's own model into `../vendor242/` (2.42, grt.tlc, R2023a, Asterope
geometry supplied at runtime on `hParams`), because a feasibility clamp that
trusts the simulation must clamp against arithmetic the wheels actually obey.
See `docs/superpowers/specs/autonomy-plan.md` §1.4.

These files are kept, unchanged, and still compiled: `test/test_vendored_model.cpp`
is the only thing that links them, and it stays as the guard that this frozen
copy remains complete. Nothing else references them. Removing the directory is
a separate decision, to be taken once someone has also ruled on the licence
note above.

**Both directories define `class kinematics`**, with different member layouts
and different input structs. No single binary may link both libraries.
```

- [ ] **Step 5: Update the project summary.** In `PROJECT_SUMMARY.md`, find the
  paragraph describing `navi_sim_ik` and its vendored kinematics and update it
  to say the simulation runs Simulink model 2.42 (grt, R2023a) with Asterope
  geometry supplied at runtime from
  `sim/src/navi_sim_ik/include/navi_sim_ik/asterope_params.hpp`, that the older
  2.41/Merope copy remains in `vendor/` only as a compiled-but-unreferenced
  regression guard, and that `test_ik_parity_242` pins the wheel outputs
  against a table generated from the vendored sources. Keep the surrounding
  prose style; do not restructure the section.

- [ ] **Step 6: Final full run.** Run:
  ```
  bash -c 'source /opt/ros/humble/setup.bash && cd /home/ole/star/Navi/sim && colcon build && colcon test && colcon test-result --verbose'
  ```
  and
  ```
  cd /home/ole/star/Navi && .venv/bin/pytest tests/ -q
  ```
  Expect: PASS on both. The laptop pytest suite is unrelated to this work and
  must be exactly as green as it was before Task 1 — run it to prove nothing
  leaked sideways.

- [ ] **Step 7: Commit.**
  ```
  cd /home/ole/star/Navi && git add sim/src/navi_sim_ik/vendor/VENDOR.md PROJECT_SUMMARY.md && git commit -m "Record that the sim now runs the rover's 2.42 IK and 2.41 is superseded"
  ```

---

## Self-review

**Spec coverage.** §1.4's requirement — the sim runs 2.42/grt with Asterope
`hParams` rather than 2.41/ert with Merope — is delivered by Tasks 1, 2 and 5,
and proved numerically by Task 4. §8's SP4 row ("sim and rover run identical
arithmetic") is what `test_ik_parity_242` asserts, and the float-widening
decision is what makes "identical" literally true rather than approximately so.
§9 rung 1 is satisfied: every test here is a pure-function laptop test with no
node harness except Task 6's deliberate live smoke. §10's speeds are the
parity grid's command set. §5's `vx_min >= -0.15` supplies the reverse row.
SP4 has no dependencies in §8 and this plan takes none.

**Placeholder scan.** No TBD, no "similar to Task N", no "add appropriate error
handling". Every code block is complete and compilable. The one intentional
blank — `// <<< TABLE GOES HERE >>>` in Task 4 Step 1 — is filled by Task 4
Step 3 from a file Task 3 Step 2 creates. An unfilled table fails immediately,
either to build (a zero-size array, under some compilers) or at test run time
(the `ASSERT_GT` guard below), so it can never be shipped silently; the runtime
`ASSERT_GT(sizeof(kGolden)/...)` is the second line of defence, against a table
that compiles but is not the harness's output.

**Type consistency across tasks.** `kAsteropeHParams` is `std::array<float, 8>`
in Task 2 and is read element-wise into `real_T hParams[8]` (i.e. `double[8]`)
in Tasks 2, 3, 4 and 5 — the widening is deliberate and tested. `kBetaDotMax`,
`kBetaDdotMax`, `kAccelerationFactor` and `kIkTimestepSeconds` are `double`,
matching the `real_T` inports they feed. `ExtU_kinematics_T` / `ExtY_kinematics_T`
are the free structs of 2.42 throughout; `kinematics::ExternalInputs` and the
`U_p` field appear only in `test_vendored_model.cpp`, which links only 2.41.
`SimIkStepper`'s public surface is unchanged, so `sim_ik_node.cpp` needs no
edit — verified by Task 5's build and Task 6's live run.

**Non-goals, stated so they are not drifted into.** The 2.41 vendor is not
deleted. `bemacontroller/` is not modified. `ground_station/` is not touched.
The `wheel3y` typo is not fixed. `WHEEL_CORNERS` remains unverified wiring
knowledge and is not changed — the model's wheel indices are the same 1..4 in
both versions, so the switch neither confirms nor disturbs it, and §11 risk 8
still stands.
