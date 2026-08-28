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
