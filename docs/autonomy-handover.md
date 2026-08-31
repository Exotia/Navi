# Autonomy stack — handover

Everything below is on the `autonomy` branch (worktree `.worktrees/autonomy`),
built by SP4–SP11 on 2026-08-30/31. Master holds only the plans and spec.
Every task passed its review; final suite sweep: GS 335 · supervisor 121 ·
teleop 88 · shaper 115 · localization 188 · autonomy 97 pure + 13 goal_relay
(domain 94) + 2 pipeline (domain 91) · navi_nav2 56 (+16 deploy-safe skips) ·
sim 143. Launcher gates green.

## The pipeline as built

```
GS (NAV row, map canvas) → /nav_request → goal_relay ⇄ Nav2 (Theta*/RPP, 0.05 m
                                          ↘ /drive_command navi_task → coordinator F0
Nav2 → /autonomy_twist → mode_supervisor → /rover_twist → twist_shaper →
/chassis_twist → bema_bridge → BEMA :21022
coordinator :21031 ⇄ navi_rpc_server :21021 (the ".18" NaVi endpoint)
elevation_mapper → tile_aggregator → traversability (holes lethal) →
/autonomy/costmap_seed → Nav2 costmaps
```

## Blocked on a human at the Orin (5 minutes, in order)

1. `sudo dpkg -i ~/orin-debs/*.deb` — fixes the nav2-msgs 1.1.18/1.1.20 skew
   that leaves bt_navigator with an undefined symbol. Staged already.
2. `sudo ip addr add 192.168.178.18/24 dev enP8p1s0` — the NaVi alias the
   coordinator calls. Better: add it to netplan so it survives reboots.
   (`start_navi.sh` adds it automatically when passwordless sudo exists.)
3. Then re-run on the Orin: the smoke launch + CPU measurement from
   `.superpowers/sdd/2026-08-31-sp9-nav2-bringup/task-7-brief.md`, and the
   offline planning test.

## Needs the camera (next daylight session)

- SP6 twist-frame MUST-DO: yaw the rover ~90°, drive forward, confirm the
  speed lands in `/localization/odom_local` `linear.x` (not `linear.y`).
- Live tile → costmap flow with real ZED data (bench-verified only).
- Enable and tune the Nav2 cloud layers (`enabled: false` today) once
  `cloud_filter` exists — it was deliberately not built (spec named it, no
  camera to test against).
- End-to-end: GS Go → coordinator Autonomous → Nav2 drives the sim mirror.

## Confirm against real hardware before trusting

- Coordinator stop semantics: goal_relay treats a `stop_seq` bump as PAUSED
  (F6 pause vs abort are indistinguishable on the wire). If the primary's
  operators use pause/resume, verify Resume works; if it aborts, change
  `on_coordinator_stop` to ABORTED.
- `startNaViTask` waypoints carry yaw 0.0 (nav_run's internal payload is
  (x, y) pairs); the coordinator ignores yaw today — confirm it stays that way.
- The RPP `use_velocity_scaled_lookahead_dist` deadlock-from-rest: disabled in
  `nav2_rover.yaml` after it froze every from-rest goal in the rung-3 test.
  Revisit with the vendor default before the first yard run.
- Wheel-corner mapping / absolute turn direction: still only verifiable with
  the rover on blocks (pre-existing gate, unchanged).

## Yard tuning (SP12 — not built, needs the rover outside)

Steering slew limits vs the shaper's measured 0.09 rad/tick, Orin CPU under
real load, traversability thresholds against real rocks, bag recording.
`fixture.py` has `elevation_from_npz()` waiting for the first real recording.

## Deploy notes

- `deploy_rover.sh --test` collides with navi_autonomy's domain-91 guard
  (pre-existing); run suites per package as the SDD ledgers do.
- Rebuild `navi_nav2` on any machine with a stale install tree (a colcon
  overlay shadowed a source file once during SP9 Task 5).
- The per-SP ledgers under `.superpowers/sdd/*/progress.md` record every
  review outcome, ruling, and parked nit.
