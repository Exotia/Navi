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
