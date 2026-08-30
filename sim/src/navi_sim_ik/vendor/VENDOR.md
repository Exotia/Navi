# Vendored kinematics

Copied verbatim from the bema controller. Do not edit these files: they are
Simulink-generated and are replaced by regenerating the model upstream.

- Origin: ssh://git@gitlab.star-dresden.de:11022/star-projekte/merope/bema/bemacontroller.git
- Commit: bd216fa
- Copied: 2026-08-28
- Paths: `src/ert_rtw/*` (less `ert_main.cpp`), `src/IkController.{h,cpp}`

Copied rather than referenced across the working tree because
`bemacontroller/` is an untracked nested repository — a clone of this project
would not have it, and the build must not depend on a directory that may not
exist.

The reason for vendoring rather than reimplementing: the simulation must run
the same arithmetic as the rover, so that a disagreement between them is a
real disagreement and not a porting artifact.

## Licence

Eight of the fourteen files under `ert_rtw/` open with this banner (copied
verbatim from the generated headers/sources):

```
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
```

Files carrying it: `kinematics.h`, `kinematics.cpp`, `kinematics_capi.h`,
`kinematics_capi.cpp`, `kinematics_data.cpp`, `builtin_typeid_types.h`,
`rtwtypes.h`, `zero_crossing_types.h`.

Files that do NOT carry it: `gpc.h`, `kinematics_capi_host.h`, `rtw_capi.h`,
`rtw_modelmap.h`, `rtw_modelmap_logging.h`, `sysran_types.h`, and
`IkController.h`/`IkController.cpp`.

This was noted at the time of vendoring. It has not been assessed by anyone
with the authority to rule on whether it affects this project's use of the
code.

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
