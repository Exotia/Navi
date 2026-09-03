# Rules and logic audit of the Navi autonomy stack

Audit date 2026-09-03 (brief dated 2026-09-02). Scope: the design as it stands in the working tree at commit 0d5692c, judged against `[ERC 2026] RULES Rev.3.pdf` and against plain engineering logic. Read-only: no code, no builds, no ROS processes. Design rationale in comments, commit messages and plan documents was deliberately not used as evidence; only what the code does counts.

## Executive summary

1. The competition gives the Traverse 20 minutes (7.4 Table 4). The stack has no notion of that clock: one unreachable waypoint can consume the whole task in the controller-stall case before the 30-retry budget is a quarter used, and a single failed leg aborts the entire run rather than deferring the waypoint, although the rules allow any visiting order (7.3.2.1.3 a).
2. The 1.0 m forced-free disc around the rover erases measured lethal cells ahead of the bumper on every map tick; Theta* then replans straight through a rock it routed around a second earlier. This is the highest expected-cost design flaw in the stack.
3. The local costmap treats unknown as free, the collision monitor has no enabled source, and observation decay forgets ground 75 s after it was last fused. On the return-to-start leg and after every spin recovery the controller drives at full speed with no obstacle knowledge beyond the wheel trail.
4. Localisation is the ZED alone and only jumps, OFF and glare are detected. Slow drift, dust and dusk degrade silently; the dead-reckoned odometry that exists is consumed by nothing.
5. In autonomous mode a lost ground-station link leaves no stop path: the software e-stop rides on rosbridge and the autonomy deadman only watches Nav2's own stream.
6. The recovery ladder (clear, spin, wait, no reverse) escapes map phantoms only. On a crater lip, a wedged wheel or in soft sand it repeats a spin up to thirty times, which the rules treat as erratic behaviour that can cancel every point (7.2.4).
7. Nothing enforces the behaviours the rules score: video can be re-enabled by one click in autonomous, the takeover threshold is any stick deflection, and the run diary that would prove autonomy lives in /tmp and is truncated at every Go.
8. The organiser provides a DEM three weeks ahead and a final grid map on the warm-up day (3.5, 7.3.2.1.4). The stack starts empty by default and has no path to use either; `--resume` across a reboot loads a map in a frame that no longer exists.

## 1. Rulebook working notes

Section numbers refer to Rev.3. Where the PDF and the code disagree, the PDF wins.

**Task shape.** The Traverse is scored 300 of 3000 points (Appendix 2). It has 15 minutes of preparation with full rover access in a dedicated area, then 20 minutes of execution (7.3.2.1.1, 7.4 Table 4). The scenario is: receive the start position and waypoint information, reach the four waypoints in any chosen sequence, reach the finish point last, return to the start, then present techniques and data (7.3.2.1.2, 7.3.2.1.3 a and b). One team is in the Main Zone at a time (7.4). The initial position and heading are drawn at the start of the trial from a designated set (7.3.2.1.4 a). Rules 7.2.4 give each task 20 to 60 minutes unless stated, and the Traverse states 20.

**What autonomy means for scoring.** Full autonomy without video feedback earns 100 percent; using a video feed halves the traverse points; using GNSS gives 80 percent (7.3.2.1.3 c to f, Table 3). The notes under Table 3 are the operative definition: autonomy is either 100 or 0 percent, "video feed used" and "GNSS used" mean the operator sees that information on the screen in the control station and manoeuvres from it, and data processed onboard and not sent to the control station keeps 100 percent. The team must present proof of autonomy to the judges (7.3.2.1.3 d, e). The general description says the operator navigates blind, without visual or spatial information, though onboard-processed support information about localisation and state is allowed, and all planning and parameter estimation must be done by the computer (7.3.2.1.1).

**What the operator may do.** Tele-operation is allowed only from the position and orientation estimate (7.3.2.1.4 b). If the rover must be moved it may only be moved back to the last successfully reached waypoint, rotated, or returned to the start, and penalties for manual intervention apply (7.3.2.1.4 c). The rover can be stopped and moved by team members when stuck or in trouble, with the judge informed first (7.3.2.1.4 e). No manual intervention is allowed during tasks except where the rules say so; any maintenance restarts the task from the start line with all points cancelled; the operator may abort at any time and keeps the points earned so far; erratic behaviour that damages infrastructure can terminate the attempt and cancel all points (7.2.4). Observers may follow the rover but cannot communicate task details to the control station and must not appear in the rover's sensors (7.2.4).

**Prior knowledge and maps.** Update Report 3 is an environmental report with a 3D model of the Mars Yard, drone photos and the locations of start points and navigation waypoints (3.5). A digital elevation model is delivered at the latest three weeks before the finals (7.3.1). The final map with grid coordinates and points of interest is handed out on the warm-up day (7.3.2.1.4, task arena). Landmarks are natural features and artificial ArUco poles; at least two are visible from the start point; some may be obscured during the traverse; teams may not place their own landmarks except by deploying them from the rover during the trial (7.3.2.1.4, task arena, and g). The judges warn that rocks, trenches, bumps and landmarks will exist that are not in the 3D model, and that driving point to point on GNSS alone risks flipping or getting stuck (7.3.2.1.3, note after Table 3). Dynamic elements such as a changed start position are announced at the start of the attempt and cannot be influenced (7.2.4).

**Environment.** Outdoor tasks must be prepared for 10 to 30 degrees, wind, drizzle, moderate rain, strong or weak sunlight; some tasks may be run at night; the yard is sandy non-cohesive soil and hard dry terrain at a variety of slope angles with many stones and boulders (7.2.3, 7.2.4).

**Communications.** Up to 100 m to the mast, line of sight may be occluded by terrain, 5 GHz is recommended, channels are assigned at the RF check, interference is not grounds for a redo, and unauthorised RF changes are penalised (5.2, 5.2.2). The rover configuration used must match the approved technical report or the task scores zero (7.3.2.1.4 d).

**Safety and emergency stop.** This PDF contains no emergency-stop specification for rovers; the requirement matrix is the external Appendix 3 spreadsheet, which was not available to this audit. The team carries full responsibility for damage (3.8). The drone preflight check (7.3.2.2.2) is the only place the rules spell out fail-safe behaviour on link loss, and it does not apply to the rover.

**Binding clarifications.** Q&A session answers are binding even when they change the rules (3.11). Several findings below end with a question that belongs in the pre-ERC briefing on 3 September (Appendix 1).

## 2. The design as implemented, in one paragraph

The ZED 2i wrapper owns map, odom and base_footprint (`localization.launch.py`). `localization_status` republishes the pose and an OK, SEARCHING or OFF state; `elevation_mapper` bins the SDK's fused cloud into a 5 cm grid and ships it as tiles; `tile_aggregator` stitches a 48 m rolling window; `traversability_layer` derives slope, step and roughness, applies decay, a startup patch, a rover heal disc, a goal heal disc and a wheel trail, and publishes a latched seed for both Nav2 costmaps. Nav2 (`nav2_rover.yaml`, two behaviour trees) plans with Theta* and Smac and drives with the rotation shim over regulated pure pursuit; its velocity leaves through the collision monitor on `/autonomy_twist`. `mode_supervisor` arbitrates manual, semi_auto, autonomous and estop onto `/rover_twist`; `twist_shaper` clamps it; `bema_bridge` sends it over msgpack-RPC to the primary and talks to the coordinator. `goal_relay` runs the mission state machine one waypoint at a time, holds at each waypoint for the operator's Resume because the coordinator enters Waiting, and tacks around glare reported by `glare_watch`. The ground station drives all of it over rosbridge with JSON on plain topics.

## 3. Front A: rules audit

Findings are ranked by expected cost: dead run, then lost waypoint, then lost time, then cosmetic.

### A1. The retry policy is blind to the 20 minute clock and aborts the whole run on one failed leg

**As implemented.** `navigate_to_pose_no_reverse.xml` wraps navigation in a RecoveryNode with 30 retries; inside it the planner gets 3 attempts per 1 Hz cycle and FollowPath gets 3 attempts, each bounded by the controller's 60 s `movement_time_allowance` (`nav2_rover.yaml` progress_checker). When Nav2 finally reports failure, `nav_run.on_goal_failed` moves the run to ABORTED and `goal_relay` sends the coordinator an abort. The ground station's mission timer (`mission_timer.py`) is a display with no connection to the run.

**Collides with.** 7.3.2.1.1 and 7.4 (20 minutes), 7.3.2.1.3 a (any visiting order), 7.2.4 (abort keeps points earned).

**Timing.** The recovery ladder costs are, per outer retry: planner-only failures about 3 s (three 1 Hz attempts) plus the round-robin action (clear about 0 s, spin 1.57 rad at 0.2 rad/s about 8 s, wait 5 s); controller-stall failures up to 3 x 60 s = 180 s plus the action. Thirty retries therefore span roughly 4 minutes in the best case and over 90 minutes in the worst. The 20 minute clock runs out during the sixth or seventh outer retry of the stall case, with 23 retries unused. Every second spent there is a second not spent on the other waypoints and the mandatory return.

**Worst case in a run.** Waypoint 2 sits behind a boulder the map shows lethal from every side. The rover spins and waits for the rest of the task, the operator has no remaining-time readout tied to the run, and the attempt ends with one waypoint reached and no return to start. Because the entire run aborts, even an operator who notices in time must build a new list and re-Go, which restarts the coordinator's task and its waypoint accounting.

**Recommendation.** Bound retries by time, not count: a per-leg budget derived from the remaining task time and the straight-line distance still to drive. On leg failure defer the waypoint and continue with the next one (the rules allow any order), then retry deferred waypoints with whatever time remains before the finish. Show the task clock and the per-leg budget on the NAV row, and make Abort automatic at a configurable time before the 20 minutes so the return-to-start is attempted.

### A2. Operator controls during the scored run: what is legal, what is enforced

The brief asks which of the tuning panel, the manual speed override and the point-to-heal gesture are legal during the task. The implemented picture is different from the brief in two places, so each control is listed as it actually exists.

**Stick takeover.** `_poll_gamepad` in `main_window.py` publishes any deflected stick in autonomous mode; `supervisor_state.on_manual_twist` treats anything above 0.002 m/s or 0.004 rad/s as a takeover, drops to manual, cancels the Nav2 goal, pauses the Nav2 lifecycle and sends the coordinator abort then startManual. Under 7.3.2.1.4 b, c and e this is a manual intervention: legal only to move the rover back to the last reached waypoint or to rotate it, with a penalty, after informing the judge. Nothing in the ground station distinguishes a deliberate takeover from a knocked gamepad, and there is no confirmation. Worst case: a bump on the controller ends the autonomous task in the judges' own state machine (the coordinator is told abort and startManual) and the run has to be restarted from a new Go. Recommendation: in autonomous, require an explicit arm (a held button or a two-step) before the stick is published, and write every takeover with its timestamp to the run diary so the intervention can be declared to the judge.

**Tuning panel.** `tuning_card.py` exposes eleven live parameters; `traversability_layer._LIVE_PARAMETERS` accepts thirteen. Changing `step_lethal_m`, `slope_lethal_deg`, `rover_heal_radius_m` (up to 3 m) or `goal_heal_radius_m` (up to 5 m) mid-run changes where the rover is willing to drive. The rules do not name this as an intervention, but 7.3.2.1.1 requires planning and parameter estimation to be done by the computer, and Table 3's note defines the forbidden case as manoeuvring the rover from what the operator sees on the screen. An operator who watches the plan view, sees the rover refuse a rock, and raises the step limit is steering by proxy. This is a jury question, not a code fact; ask it at the 3 September briefing and record the answer (3.11 makes it binding). Until then, treat any mid-run retune as an intervention and log it. Note also that the panel's spin-box maximum for the step limit is 1.0 m and the rover accepts up to 1.0 m, while the belly clearance is 0.282 m; the physical ceiling is a tooltip, not a limit.

**Point-to-heal.** There is no separate heal gesture. Clicking the map (`nav_map_view.point_clicked`) appends a waypoint, Go is disabled during a run, and the only in-run heal is the fixed 1.4 m disc `goal_relay._publish_active_goal` places around the judges' own waypoint. That disc is automatic and rules-neutral, but see B3 for what it erases.

**Speed override.** `speed_card.py` caps the gamepad only and is hidden in the autonomous view (`main_window.py` line 1128). Nav2's speed is the fixed `desired_linear_vel: 0.2` in `nav2_rover.yaml`. There is no live autonomous speed control, so nothing to rule on; the brief's premise does not hold.

**Video.** Entering the autonomous view calls `_request_rover_video(False)` (line 1144), which matches Table 3's "video feed not used". But `_on_stream_requested` refuses the toggle only in the semi_auto and simulation views; in the autonomous view a press of "Start video" falls through to `_request_rover_video(True)` and the rover streams H.264 to the control station. One click separates 100 percent from 50 percent of the traverse points. Recommendation: refuse the toggle in the autonomous view with the same one-line notice the semi_auto view uses, and record on the rover side whether a video request was ever accepted during a run (`video_sender` knows) so the proof of autonomy can state it.

**Pause and Resume at every waypoint.** `nav_run.on_goal_succeeded` moves the run to PAUSED after each waypoint because the coordinator enters Waiting with movement disabled; the operator must press Resume, which re-arms (RESUME_TASK, wait for an observed Autonomous, up to 12 s). Pressing a control-station button is not a rover intervention under 7.2.4, so this is legal, but with four waypoints and a finish it inserts four human-paced gaps into a 20 minute task, each costing the coordinator's roughly 5 s re-arm plus reaction time. Recommendation: an auto-resume option that sends Resume the moment the coordinator reports Waiting, with the operator able to veto.

**Map save, load and clear.** `map_row` is visible only in semi_auto (line 1117). Fine during the run.

### A3. Map policy discards the prior knowledge the organiser supplies and `--resume` is unsafe across a reboot

**As implemented.** `start_navi.sh` starts `elevation_mapper` with an empty grid unless `--resume` names a saved map or `latest`. The map frame is born at the ZED's boot pose and the ZED persists no area memory (`site_frame.reexpress_at_lock_pose` docstring, `zed_front.yaml`). A saved map is stored in that frame with no record of it (`map_store.save` writes elevation, origin, resolution and voxels, no frame anchor). Both planners drive through unknown ground at cost 25 on the global map (`unknown_plan_cost`) and as free on the local map.

**Collides with.** 3.5 and 7.3.1 (DEM and 3D model three weeks ahead), 7.3.2.1.4 task arena (final grid map with POIs on the warm-up day). Nothing in the rules forbids prior maps; the organiser hands them out.

**Chains.** When the rover is rebooted between the preparation phase and the attempt and `--resume latest` is used, the loaded grid is expressed in the previous boot's frame; the new frame is born wherever the rover now stands, so every cell is misregistered by the difference between the two boot poses, with no warning (`_load_startup_map` warns only on a missing or corrupt file). When the default empty start is used, the craters and embankments the DEM shows are unknown until the camera sees them, and the return-to-start leg is planned through ground decay has forgotten (see B4).

**Worst case.** A misregistered resume puts a phantom crater rim across the start line; the startup patch and rover heal clear a disc but the first plan routes around a wall that is not there, or through a real one the loaded map placed elsewhere.

**Recommendation.** Refuse `--resume` unless the saved map carries a site-frame anchor that matches the current lock (store the `SiteTransform` with the map, or store the map in site coordinates). Add an import path for the organiser DEM as a static prior layer in the site frame, converted once the site anchor is locked; keep camera observations authoritative where they exist.

### A4. Design premises the rulebook does not support

**"A path is guaranteed to exist"** (the 30-retry premise, both behaviour trees). The rules say the opposite: rocks, trenches and bumps not in the model (7.3.2.1.3 note), stones and boulders at a variety of slopes (7.2.3), start position drawn at the trial (7.3.2.1.4 a). A waypoint may be reachable only from a side the map has not seen, or not reachable at 0.25 m step tolerance at all.

**"The yard is static during a run"** (`stamp_wheel_trail`, permanent trail). Traverse is one team at a time (7.4) and observers must keep out of the sensors (7.2.4), so this holds in practice, but the trail is a set that is never pruned and survives a Pause, an Abort and a new Go; only a node restart clears it. Across the 15 minute preparation drive plus the task, the trail can cover ground the judges then place a dynamic element on.

**"The operator is watching"** (Resume at every waypoint, Abort under the clock, the takeover path). The rules put the operator in a separate control station without sight of the rover (7.2.4) and make the operator responsible for aborting; nothing on the rover detects an absent ground station.

**"Autonomous means the same to us and to the judges."** The coordinator on the primary (not in this repo) is told startNaViTask, pause, resume and abort; its own state (PrepareAutonomous, Autonomous, Waiting) is what the judges' side reads through the primary. Every takeover and localisation halt sends abort then startManual. If the judges score autonomy from the primary's log, each of those is an intervention record even when the rover never moved.

### A5. Proof of autonomy is a truncated file in /tmp

**As implemented.** `RunLog` writes to `/tmp/navi_last_ride.log` and `start()` opens it with mode "w" at every accepted Go (`run_log.py` line 32). `/tmp` does not survive an Orin reboot.

**Collides with.** 7.3.2.1.3 d and e: each team is responsible for presenting proof of autonomy. The diary is the only complete decision record the stack produces.

**Worst case.** The run aborts at waypoint 3, the operator re-Goes with the remaining waypoints, and the record of the first three legs is gone. The judges ask for proof; the team has the second half.

**Recommendation.** One file per run id under a persistent directory, never truncated, plus a machine-readable summary (waypoints sent, reached, takeovers, video requests, mode changes with timestamps) the presentation can cite.

## 4. Front B: design-flaw audit

### B1. The rover heal disc erases measured obstacles ahead of the bumper

**As implemented.** `traversability_layer._on_map_inner` applies, in order, the seed, the startup patch, decay, `heal_goal_patch` around the rover's current cell with radius `rover_heal_radius_m` = 1.0 m, the goal heal, then the wheel trail. `heal_goal_patch` sets every cell in the disc to 0, measured LETHAL included. The footprint is 0.46 m long by 0.445 m wide with 0.03 m padding, so the disc reaches 0.51 m beyond the front bumper. Both costmaps consume this seed; the local voxel layer and the collision monitor's point source are disabled (`enabled: false`), so no other layer can put the obstacle back.

**Failure chain.** When a 0.3 m rock is 1.5 m ahead, the seed marks it lethal and Theta* routes around it. When the rover has closed to 1.0 m, the rock is inside the disc and reads 0. At the next 1 Hz replan Theta*, which minimises length with traversal weight 2.0, finds the straight line through the rock cheaper than the arc and replans through it. RPP's collision detection reads the same local costmap and sees free. The rotation shim and RPP steer onto the new path. At 0.2 m/s the rover reaches the rock in 2.5 s, which is inside the interval before the next seed could disagree, and it cannot disagree because the rock stays inside the disc until the bumper touches it.

**Worst case.** High-centred on a rock (belly clearance 0.282 m against a 0.3 m rock) or a wheel over a crater lip. A physically stuck rover with the ladder of B6 below spinning on it. Under 7.2.4 the judges can end the attempt.

**Recommendation.** Restrict the rover disc to cells that are UNKNOWN (the semantics of `clear_startup_patch`), or to cells the wheel trail already proves, and never to measured LETHAL; if a forced-free disc around the rover is kept for the start-pose check, apply it only in the planner's start validation (a small radius equal to the footprint's inscribed circle) and not to the local costmap the controller drives on. The same applies to the 1.4 m goal disc: heal UNKNOWN and phantom-height cells, not measured steps.

### B2. Blind driving: unknown is free locally, there is no live obstacle source, and decay forgets the way home

**As implemented.** `local_costmap.track_unknown_space: false` (unknown cells read free), `voxel_layer.enabled: false`, `collision_monitor.points_filtered.enabled: false`, `observation_decay_s: 75` turns any cell whose fusion stamp is older than 75 s to UNKNOWN, `unknown_plan_cost: 25` prices unknown on the global map, both planners `allow_unknown: true`, and RPP's cost-regulated speed scaling reads 0 cost for free cells and keeps full speed.

**Chains.**

1. Return leg. The camera looks forward (ROI bottom half, 8 m mapping range, `zed_front.yaml`), so ground behind the rover is never re-fused. At 0.2 m/s the rover is 15 m from anything it saw 75 s ago. Every crater rim beside the outbound track is UNKNOWN by the time the finish leg is planned; the global planner pays 25 per cell to cross it and 0 along the wheel trail; the trail is a 0.4 m disc, narrower than the 0.49 m half footprint, so a plan hugging the trail's edge overhangs forgotten ground, and the local map calls that ground free.
2. After a spin recovery. The rover faces a new heading whose ground it has not mapped; the 1.0 m disc is free by fiat; the plan crosses unknown at 25; the controller drives at 0.2 m/s; the camera needs about 1 Hz cloud plus 300 ms derive plus a 1 Hz replan, roughly 2 to 3 s, before a lip can appear in the seed, by which time the rover has moved 0.5 m and the lip may already be inside the disc of B1.
3. Frozen seed. If `traversability_layer` or `tile_aggregator` dies, the latched seed stays on its last value and Nav2 plans on it indefinitely; no stamp check exists on the static layers and nothing on the ground station watches seed age.

**Worst case.** A crater lip 1 to 2 m off the outbound track on the return leg, or on the new heading after a spin: dead run under 7.2.4, and the finish, which must be reached last, is lost.

**Recommendation.** Decay only cells the camera could have re-observed (inside the frustum and within range in the last N seconds); keep lethal cells far longer than free ones (a lethal cell that was measured once is worth more than a free cell that was not re-confirmed). Track unknown space in the local costmap and let RPP's cost regulation slow the rover through it; if the spin collision checker is the reason unknown was made free, give the behaviour server its own costmap topic. Enable the collision monitor with a live point source before the yard; until then, cap the speed on any plan segment that crosses unknown. Add a seed-age watchdog that pauses the run when the seed stamp is older than a few seconds.

### B3. The goal heal disc erases obstacles at the judges' waypoint

**As implemented.** `goal_relay._publish_active_goal` publishes each real waypoint; `traversability_layer` forces a 1.4 m disc around it free every tick; the general goal checker accepts arrival at 0.25 m.

**Chain.** The judges place a waypoint next to a boulder (7.3.2.1.3 note: not everything is in the model). Inside 1.4 m every cell is free, the last 1.65 m of approach are driven blind, and the boulder within 0.25 m of the point is invisible.

**Worst case.** The rover contacts the boulder at the waypoint; if it beaches, the ladder of B6 spins on it.

**Recommendation.** Heal only UNKNOWN and phantom-height cells around the goal; let Smac's tolerance handle a goal that lands on a measured obstacle by shifting it, which it already does, and raise `goal_reached_tol` awareness on the NAV row so the operator sees when a goal was shifted and by how much.

### B4. Interacting timers: the diagram and the gaps

| Mechanism | Value | Where |
| --- | --- | --- |
| Coordinator arm timeout | 12 s (PrepareAutonomous to Autonomous about 5 s) | `nav_run.ARM_TIMEOUT_S` |
| Coordinator state poll | 1 Hz publish, 5 Hz read | `bema_bridge.STATUS_HZ`, `goal_relay.TICK_HZ` |
| Localisation grace | 3 s SEARCHING, OFF immediate, OFF after 2 s silence | `supervisor_state`, `localization_status` |
| Pose-jump reacquire | 15 consecutive poses, about 1 s | `tracker.REACQUIRE_POSES` |
| Progress checker | 60 s, 0.25 m or 0.20 rad | `nav2_rover.yaml` |
| Observation decay | 75 s since last fusion write | `traversability_layer` |
| Glare detour | 30 s each, 4 per leg | `goal_relay` |
| Glare heartbeat | 5 s | `glare_watch.HEARTBEAT_S` |
| Planner retries | 3 per cycle at 1 Hz, Smac 5 s max | BT, yaml |
| FollowPath retries | 3, each up to 60 s | BT, yaml |
| Outer retries | 30 (ten clear, spin, wait laps) | BT |
| Spin | 1.57 rad at 0.2 rad/s, about 8 s | yaml behavior_server |
| Wait | 5 s | BT |
| Autonomy deadman | 0.5 s on `/autonomy_twist` | `supervisor_state` |
| Bridge deadman | 1.0 s on `/chassis_twist` | `bema_bridge` |
| Smoother timeout | 1.0 s | yaml velocity_smoother |
| Collision monitor stop timeout | 2.0 s | yaml |
| Drive link | 0.3 s RPC timeout, reconnect backoff 1, 2, 4, 5 s | `bema_session` |
| Tile keepalive | 1 s minimum interval, one keepalive tile per tick | `tiles.py` |

**Where two fight.**

- Decay 75 s against the 60 s stall allowance is the intended order, but decay is measured from the fusion stamp, not from the stall. A rover that has been stationary for 45 s while the planner thinks, then spins for 8 s, then waits 5 s, has 58 s on the progress checker and cells behind it at 75 s. The spin's collision checker runs on the local costmap where unknown is free, so it will turn; the global replan after the spin sees the surroundings as unknown at 25 and picks a new corridor. The result is a rover that changes its mind about which way out every recovery lap. Lost time, and B2's blind drive on each new heading.
- The autonomy deadman (0.5 s) against Nav2's own quiet periods. The smoother stops after 1 s without input and the collision monitor after 2 s; during Smac's 5 s planning window `/autonomy_twist` can go silent, the supervisor publishes zeros and sends a chassis stop, then resumes when twists return. Each cycle passes through `bema_session.stop` (F1 zero, F2) and the shaper's retained geometry. Functionally safe; on the ground station the DEADMAN pill flickers during every plan, which trains the operator to ignore it.
- Glare detour 30 s against the progress checker 60 s: a detour that cannot be reached is abandoned at 30 s by `goal_relay`, before the checker could fail it, so the checker never sees a detour fail. Fine. But four detours plus four cancels plus four replans per leg is up to two minutes and twenty seconds of a leg, and with five legs that is more than half the task.
- Arm timeout 12 s against a coordinator that dropped to Idle. `on_coordinator_stop` moves the run to PAUSED and lights Resume; Resume sends F5, which the coordinator refuses from Idle; the run waits the full 12 s and aborts. The ground station invites a Resume that cannot succeed.

**Where a stall is invisible to all of them.**

- Wheels turning, rover not moving, ZED drifting. The progress checker watches `/localization/odom_local`, which is the ZED's odometry. A camera whose VIO slides (dusk, low texture, dust) reports motion; the checker is satisfied; decay keeps refreshing the cells in view; nothing trips. The rover digs in sand or grinds a wheel until the operator notices, and the operator's screen shows the pose moving.
- Drive link down in autonomous. `bema_session._mark_down` closes both sockets and backs off; the wheels stop by the bridge's deadman; Nav2 keeps commanding; the checker trips after 60 s; FollowPath is retried three times with a local clear; only then a recovery. `goal_relay` does not read `/drive_status.connected` or `lease`; it reads only the coordinator state integer, and only while STARTING. On reconnect the coordinator may no longer be Manual or Autonomous, in which case `bema_session` never re-sends the movement enable (F7 True is gated on state 3 or 5) and the wheels stay dead while the NAV row shows RUNNING.

**Recommendation.** Feed `/drive_status` into `goal_relay` and pause the run on `connected == false` or on a coordinator state that is not Autonomous for more than one status period; on reconnect require an observed Autonomous before resuming. Cross-check commanded motion against reported motion (the dead-reckoned odometry already exists, see B5) and trip a slip alarm when commanded distance exceeds observed distance by a threshold over 10 s. Show seed age, decay coverage and drive-link state on the NAV row.

### B5. Localisation degrades silently and its fallback is unwired

**As implemented.** `tracker.py` rejects a pose that jumps more than 2 m, moves faster than 5 m/s, or has |z| over 20 m, and calls the wrapper OFF after 2 s of silence. `glare_watch` reacts only when 20 percent of a half frame is saturated and the halves differ by a factor of three. `twist_odometry` publishes `/odom/twist` with a growing covariance; no node, launch file or parameter in `rover/src` or `ground_station` subscribes to it. The wheel-odometry plan (`docs/superpowers/plans/2026-09-02-wheel-odometry.md`) reports itself as planned and not started.

**Chains.**

1. Slow drift. Dusk, dust on the lens, motion blur at the 0.5 rad/s shim turns on 15 fps, or a featureless sand slope makes the VIO slide by a few centimetres a second without any status change. The map is written in the map frame; new fusion lands beside old fusion; steps appear at the seams; the seed goes lethal in a ring; the rover heal disc erases the ring under the rover; the planner finds a corridor through the phantoms one tick and not the next. The wheel trail marks ground the rover believes it drove over, which is now offset from the ground it actually drove over. No detector exists for any of this.
2. Detected loss. SEARCHING for more than 3 s or OFF puts the supervisor in manual, cancels the goal, pauses Nav2 and stops the chassis; `nav_run.on_mode_status` aborts the run and the coordinator is told abort. There is no hold-and-wait: a 4 s VIO outage that recovers on its own has already ended the run, and `on_mode_request("autonomous")` is refused until localisation reads OK again. The operator must then press Autonomous and Go with a new list from index 0, which restarts the coordinator's task. Recovery by hand is the manual intervention 7.3.2.1.4 c penalises.
3. Glare in the sky half. The SDK's region of interest is the bottom half of the frame (`zed_front.yaml`), so tracking never uses the top half. `glare_watch` measures the whole left image. A low sun above the horizon on one side saturates the top half on that side and can exceed 20 percent of the half frame; `goal_relay` tacks 2 m sideways for up to 30 s, four times, for a sun that tracking never looked at. Conversely, glare is consulted only when a leg is dispatched and after a detour succeeds (`_advance_toward_real_goal`); a leg that curves into the sun mid-way gets no detour at all, and the loss it was designed for happens anyway.
4. Loop closure. `area_memory: true` and `reset_odom_with_loop_closure: true` let the SDK jump both map and odom when it closes a loop. A jump over 2 m is rejected as a pose jump for about 1 s, within the supervisor's 3 s grace, so the run survives; a jump under 2 m is accepted at once. The elevation grid is not re-fused wholesale on a closure, so old cells stay where the pre-closure frame put them: a controlled version of chain 1, triggered exactly when the rover returns toward the start, which is the leg the rules require last.

**Worst case.** A rover that believes it is 0.5 m from where it is, driving on a map of phantoms toward a waypoint the judges measure it as having missed, or a run ended by a 4 s outage at waypoint 3.

**Recommendation.** Consume `/odom/twist`: compare its distance over the last 10 s against the ZED's and declare degraded localisation (not lost) when they diverge beyond a threshold, and treat degraded as "slow down and stop mapping", not "abort". On SEARCHING, hold the run (cancel the goal, keep the mission state) for a configurable window before aborting, and let a return to OK resume automatically at the same waypoint index. Evaluate glare on the tracking ROI only, and re-evaluate it on every feedback callback, not only at dispatch. Disable loop-closure odom resets during a run or handle the closure by re-anchoring the grid.

### B6. The recovery ladder escapes phantoms only and worsens physical stuck states

**As implemented.** The RoundRobin is clear both costmaps, spin 1.57 rad, wait 5 s; `behavior_plugins` are spin and wait only; `allow_reversing: false`; the collision monitor polygons start at x = +0.10 m. There is no reverse, no assisted teleop and no "stop and ask".

| Stuck geometry | What the ladder does | Escapes? |
| --- | --- | --- |
| Map phantom (drift seam, noise blob) | Clear removes it, spin re-observes, decay fades it | Yes, this is the case it was built for |
| Blind pocket wider than about 1.3 m | Spin in place, replan out | Yes |
| Blind pocket narrower than the footprint diagonal | Spin scrubs the walls; no reverse | No |
| Crater lip, front wheel over the edge | Spin drags the wheel along the lip; the 1.0 m disc says the lip is free | No, and it makes it worse |
| Wheel wedged or belly on a rock | Spin grinds; wait does nothing | No |
| Soft sand, wheels digging | Progress fails after 60 s, spin digs deeper, repeat 30 times | No, and it makes it worse |

**Collides with.** 7.2.3 (non-cohesive sand, boulders, slopes), 7.2.4 (erratic behaviour and damage can end the attempt), 7.3.2.1.4 e (a human may lift the rover, with penalty, judge informed).

**Worst case.** Thirty spins on a crater lip in front of the judges.

**Recommendation.** Detect the physical cases before recovering: no-progress with the commanded distance far above observed distance is slip; tilt from the ZED IMU beyond a threshold is a lip. On either, stop and report to the ground station rather than spin. Add a short blind reverse (0.2 m at 0.05 m/s along the wheel trail, which is ground the rover just proved) as the first rung for the wedge and pocket cases; the trail is the one place a reverse is not blind. Cap the ladder at two laps and hand the decision to the operator, who can then invoke 7.3.2.1.4 e deliberately.

### B7. Risk posture versus physics

- `step_lethal_m` 0.25 against a belly clearance of 0.282 m leaves 3.2 cm of margin, less than the 5 cm cell and less than the depth noise the fitted slope exists to average. Because `step` is a maximum absolute difference, a 0.24 m hole is also drivable everywhere outside the 3 m rover-relative radius where `drop_lethal_m` 0.14 applies, on a wheel of 0.125 m radius. Recommendation: 0.20 m step, and apply the drop limit globally, not only near the rover.
- `slope_lethal_deg` 30 against a static tip of about 47 degrees is a fair margin on hard ground; on non-cohesive sand (7.2.3) side slip on a 30 degree cross-slope precedes tipping. The slope is fitted over a 0.4 m window, which halves a short 0.2 m bank. The costmap has no notion of cross-slope versus along-slope. Recommendation: 25 degrees on the coarse global map, keep 30 locally, and add a lateral-tilt guard from the IMU.
- Unknown at 25 of 100 on the global map, and free on the local map, means a global plan that enters unseen ground arrives at the controller with zero cost. RPP's cost-regulated scaling keeps 0.2 m/s and its collision check finds nothing. The only brake is the camera, 2 to 3 s later. This is B2's mechanism; the price of unknown is the wrong knob because it only changes the route, not the speed on it.

### B8. State-machine honesty

- Detours are invisible to `nav_run` and to the NAV row: `_on_detour_feedback` writes only the diary, so the operator sees RUNNING with a frozen distance for up to 30 s per detour, which reads as a stall and invites an Abort.
- Waypoint reached is rendered as PAUSED with the message in the red error pill (`nav_row._refresh_status` colours any error BAD). A success looks like a fault at every waypoint.
- After a goal failure the supervisor stays in autonomous while the run is ABORTED and the coordinator is Idle; the header says AUTONOMOUS. After a localisation halt the supervisor is manual, the NAV row hint says "press Autonomous first", and the request is refused until OK, with no countdown shown.
- On a coordinator stop (`stop_seq`) the run goes PAUSED and Resume lights up although the coordinator may be Idle and will refuse F5; the failure arrives 12 s later as "coordinator did not reach Autonomous".
- An operator Pause mid-detour is handled: cancel clears the callbacks, Resume re-dispatches the same index without refilling the detour budget. Glare during a recovery is not handled at all (see B5 chain 3). Goal reached during a spin resolves correctly through the BT's main branch on the next tick.
- `/nav_status.distance_remaining_m` adds straight-line legs ahead to Nav2's path length; on a detour it is the previous real-goal value. The ETA is derived from 0.2 m/s and never from the actual speed.

**Recommendation.** Publish detour state on `/nav_status` (a `detour` field with target and elapsed), colour the waypoint-reached hold as OK, gate Resume on a coordinator state that can accept it, and show remaining task time beside the ETA.

### B9. Silent degradation of the camera as a map source

Dust on the lens produces near-field returns (min depth 0.3 m) that read as steps directly in front of the rover; `mask_floating_cells` drops them if they hang more than 0.35 m above their neighbours, otherwise the 1.0 m disc erases them. Dust also produces holes; holes are NaN, NaN is unknown, unknown is passable at 25 globally and free locally. Both directions therefore degrade toward "drive on". Dusk lowers the fused cloud density and the 20th percentile height per cell becomes a single point per cell; the slope fit needs 6 finite cells, below which the cell is unknown, again passable. There is no measure of map quality (points per cell, fraction of the frustum that returned depth) surfaced anywhere, so the operator cannot see the camera going dim.

**Recommendation.** Publish per-tick map quality (finite fraction inside the frustum, median points per cell) and let the supervisor treat a collapse as a localisation warning; slow the rover when quality falls.

### B10. Assumption inventory

| Assumption | Detector | Response | Status |
| --- | --- | --- | --- |
| A path to every waypoint exists | none | 30 retries then abort run | hope |
| The yard is static during a run | none | permanent wheel trail | hope, rules make it likely |
| The operator is watching | none on the rover | none | hope |
| The ground-station link holds | manual: 1 s deadman; autonomous: none | manual: stop; autonomous: continue with no stop path | half |
| The drive link holds | bridge 1 s deadman, backoff | wheels stop; Nav2 unaware until 60 s | half |
| ZED tracking is honest | jumps over 2 m, speed over 5 m/s, OFF | abort run, mode manual | detects the loud failures only |
| ZED tracking is not drifting | none | none | hope |
| The map is fresh | none (no seed-age check) | none | hope |
| The map is well registered | site-anchor residual at lock | operator reads it | operator |
| The camera sees the ground ahead | none | heal disc assumes it | hope |
| The coordinator is alive and Autonomous | 1 Hz state poll | only while arming | half |
| Nav2 lifecycle resumes when asked | goal rejection | abort run | detects, then over-reacts |
| The Orin holds 1 Hz on the derive | none (Orin numbers "not taken") | none | hope |
| The clock is sane | none; ages use cloud header stamps with node-clock fallback | none | low risk, same host |
| The site anchor is right | residual, two landmarks cannot catch a swap | operator | operator |

## 5. Compliant or safe by luck

These are places where the behaviour the team relies on is true today but nothing enforces it.

- **No video in autonomous.** True because entering the autonomous view disables the stream; one press of "Start video" re-enables it. Refuse the toggle in that view.
- **Autonomous speed cap of 0.2 m/s.** Set only in `nav2_rover.yaml`; the shaper backstop is 0.5 m/s and the supervisor does not clamp. A yaml edit is the whole cap.
- **Waypoints at least 1 m apart.** `goal_reached_tol` 1.0 m assumes the shortest leg is longer than a metre; the ground station accepts any list. Two waypoints 0.8 m apart make the second "reached" on arrival at the first, and the coordinator is told so.
- **Emergency stop reachability.** `/estop_request` travels over rosbridge; in autonomous mode a lost link stops nothing and no rover-side watchdog exists for the ground station. Whether a hardware stop exists is outside this repo; if it does not, an autonomous rover on a dead link can only be stopped by a person, which 7.3.2.1.4 e permits with a penalty. Add a ground-station heartbeat and pause autonomy after a few seconds without it.
- **Movement enable.** `bema_session` sends F7 True itself whenever the coordinator is Manual or Autonomous and it holds the lease; the coordinator's own enable never lands. This works because the primary's state machine happens not to fight it.
- **Takeover threshold.** 0.002 m/s and 0.004 rad/s against a gamepad whose smallest output is 0.005 m/s; any drift in the gamepad's own centring that escapes the 0.15 deadzone is a takeover.
- **Physical ceilings in the tuning panel.** The belly clearance (0.282 m) and the footprint's inscribed circle (0.445 m) are tooltips; the rover accepts a step limit of 1.0 m and a trail radius of 1.0 m.
- **Local costmap unknown is free** so that Spin can run inside a decayed blank. Spin's collision checker and the controller now share one costmap with one convention; the convention chosen protects the recovery and exposes the drive.
- **Latched seeds with no age.** A dead perception node freezes both costmaps on their last value with no visible symptom on the rover or the ground station.
- **Orin timing.** The derive is 300 ms on a laptop; the Orin measurements are recorded as not taken. If the Orin exceeds the 1 s tick, the latched map subscription (depth 1) drops ticks silently and decay ages advance faster than the seed refreshes.
- **Behaviour server yaml.** A `backup` plugin block remains in `nav2_rover.yaml` while `behavior_plugins` lists spin and wait only; inert today, a trap if someone adds it back to the list.

## 6. Ranked list of findings

| Rank | Finding | Class | Section |
| --- | --- | --- | --- |
| 1 | Rover heal disc erases measured obstacles ahead of the bumper | dead run | B1 |
| 2 | Unknown free locally, no live obstacle source, decay forgets the way home | dead run | B2 |
| 3 | Recovery ladder spins on lips, wedges and sand | dead run | B6 |
| 4 | Silent localisation drift with the fallback unwired | dead run | B5 |
| 5 | No stop path for an autonomous rover on a lost link | dead run (safety) | A5 note, section 5 |
| 6 | Goal heal disc erases obstacles at the judges' waypoint | dead run | B3 |
| 7 | Retry policy blind to the 20 minute clock; one failed leg aborts the run | lost waypoints | A1 |
| 8 | Drive-link degradation invisible to the run | lost waypoint | B4 |
| 9 | Localisation halt ends the run; manual re-Go from index 0 | lost waypoint | B5 chain 2 |
| 10 | Any stick deflection is a takeover that aborts the coordinator task | lost waypoint | A2 |
| 11 | Step 0.25 m against 0.282 m belly; drop limit only near the rover | lost run | B7 |
| 12 | No DEM prior; `--resume` misregisters across a reboot | lost time or run | A3 |
| 13 | Glare detour on the sky half, evaluated only at dispatch, up to 140 s per leg | lost time | B5 chain 3, B4 |
| 14 | Video re-enable one click away in autonomous | lost 50 percent of traverse points | A2 |
| 15 | Resume at every waypoint is human paced | lost time | A2 |
| 16 | Resume offered on a coordinator that will refuse it | lost time | B4, B8 |
| 17 | Tuning panel mid-run: legality undefined, physical ceilings unenforced | jury question | A2 |
| 18 | Run diary truncated per Go in /tmp | proof of autonomy | A5 |
| 19 | Path jitter from the moving disc and decay boundaries | lost time | B4 |
| 20 | Waypoint reached shown as a red error; detours invisible on the NAV row | cosmetic | B8 |

## 7. Questions for the 3 September briefing

Answers are binding under 3.11 and should be filed beside this report.

1. Does changing traversability thresholds from the control station during the attempt count as manual intervention or as operator parameter estimation under 7.3.2.1.1?
2. Is a control-station Resume at each waypoint (the coordinator's Waiting hold) acceptable within "fully autonomous", given the rover does not move and no spatial information is used?
3. Is watching a video feed without manoeuvring "video feed used" under Table 3, or is only manoeuvring from it?
4. What arrival tolerance do the judges apply at a waypoint, so `goal_reached_tol` and the checker's 0.25 m can be set against it rather than against each other?
5. Is a prior map built from the organiser's DEM and 3D model permitted as an onboard planning layer? (Nothing in Rev.3 forbids it.)
6. What is the rover emergency-stop requirement in Appendix 3, and does a software stop over Wi-Fi satisfy it?
