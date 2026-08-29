---
name: sim-e2e
description: Laptop end-to-end of the semi-autonomous view with mocks (no rover) — mapper + synthetic cloud on domain 91, Gazebo on 42, model counts, clear/save/load, chase-camera frame, clean teardown. User-invoked only.
disable-model-invocation: true
---

# /sim-e2e

Run `bash .claude/skills/sim-e2e/scripts/e2e.sh [seconds]` (default 120) and report its printed summary: terrain/obstacle model counts vs tiles, "already exists" count (must be 0), clear→0 time, the frame path (look at it: black rover, orange ground, light-sand blocks, no spikes), and the teardown check (domains 42/91 empty, no gzserver).

Rules the script encodes: mocks on ROS_DOMAIN_ID=91, never domain 0, never /manual_twist (uses /sim_test_twist), kills by PID / `pkill -x`, logs under the session scratch dir.
