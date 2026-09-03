# Asterope (Navi) against AGH kalman_robot, second pass

Read-only comparison of `/home/ole/star/Navi` (ours) against
`/home/ole/star/Navi_Compare/kalman_robot` (AGH Space Systems, the reference).
Nothing was built, run or published to. Paths below are relative to those two
roots.

## Executive summary (ten lines)

1. The largest structural difference is not in Nav2 at all: AGH treat a Nav2
   abort as the *beginning* of a recovery, we treat it as the end of the run.
2. Their supervisor drives the rover to a remembered free position with the
   obstacle layers switched off, then re-sends the same goal; ours has no
   answer to an abort except telling the operator the mission failed.
3. They give Nav2 three retries and escalate in about fifteen seconds; we give
   it thirty and churn Clear/Spin/Wait for minutes before dying.
4. They validate and repair every goal against the live costmap before sending
   it; we send the operator's point raw and fake the map underneath it instead.
5. They have an explicit mechanism for goals outside the rolling costmap
   (partial goals, chained); we have none, and our window is 48 m rolling.
6. Their controller structurally cannot fail; ours may abort after 0.5 s of
   `failure_tolerance` against a local costmap that has no live sensor input.
7. Two of their settings we are missing are free to copy and both match the
   "terminates for no visible reason" symptom: `wait_for_service_timeout` and
   `action_server_result_timeout`.
8. Their wheel realignment is handled below the controller and defers the
   drive; our shaper scales the whole twist to as little as a twentieth for
   three seconds at exactly the rotation-shim hand-off.
9. They monitor topic liveness and can respawn; we have no health monitor and
   no respawn, so a dead `goal_relay` is silent.
10. Our velocity arbitration, e-stop, deadman chain and operator telemetry are
    better than theirs and should not be traded away for any of the above.

## Scope

The first comparison pass already adopted observation decay (75 s), plane
fitted slope lethality (30 degrees), the coarse planning costmap with unknown
priced at 25, and the no-physics simulation philosophy; it deferred the
robot_localization EKF, whose plan is
`docs/superpowers/plans/2026-09-02-wheel-odometry.md`. None of those five are
re-derived here. Where a finding touches one of them it says so and adds only
what the first pass missed (see B6 and B10).

---

# A. Differences that plausibly explain our yard failures

Ranked by strength of evidence, strongest first.

## A1. A Nav2 abort ends our mission; on their side it starts a recovery

**(a) What each side does.**

Ours: `rover/src/navi_autonomy/navi_autonomy/nav_run.py:269-273` is the whole
of our response to a failed Nav2 goal. `on_goal_failed` calls `_abort`
(`nav_run.py:281-289`), which sets the run to `ABORTED`, records the reason and
queues `ABORT_TASK` at the coordinator. The node side
(`rover/src/navi_autonomy/navi_autonomy/goal_relay.py:350-356`) only logs it.
There is no retry, no alternative goal, no attempt to move the rover somewhere
a plan could exist. The operator has to press Go again from the start.

Theirs: `kalman_supervisor/kalman_supervisor/modules/nav.py:194-302` is a
recovery state machine wrapped around Nav2. On `STATUS_ABORTED`
(`nav.py:210-233`) it asks `position_history` for the most recent waypoint the
costmap still calls free (`modules/position_history.py:53-67`), falls back to a
spiral search for the nearest free cell around the robot
(`modules/map.py:207-260`), pushes that point one metre further from the robot
so it is not sitting on the costmap edge (`nav.py:236-244`), **disables the
obstacle layers on both costmaps** (`nav.py:246-250`, via
`modules/map.py:76-109` and the layer names in
`kalman_supervisor/config/supervisor.yaml:16-20`), drives there
(`nav.py:252-264`), re-enables the layers (`nav.py:266-274`) and then re-sends
the *original* goal (`nav.py:276-287`). If recovery is genuinely impossible it
reports the goal as `SUCCEEDED` rather than failing the mission
(`nav.py:225-233`, and the same choice again at `nav.py:119-129` for a goal with
no free space near it).

**(b) Does it explain a yard failure?** Yes, directly, and it is the single
best match for the "plans correctly and then terminates" family. Every abort
Nav2 can produce - a plan that went empty, a controller that gave up, the
recovery budget exhausted - arrives at a supervisor with exactly one response,
which is to stop.

**(c) Cost of adopting.** Cheap to medium, and right for us. It needs no new
sensor and no new Nav2 feature: a subscription to `global_costmap/costmap`, a
ring buffer of poses the rover has actually stood on, and one
`SetParameters` call against `local_costmap/local_costmap` and
`global_costmap/global_costmap`. Note that our seed layer is named
`static_layer` on both costmaps (`rover/src/navi_nav2/params/nav2_rover.yaml:339`
and `:405`), so "disable the obstacle layers" for us means disabling the static
layer, which is a bigger statement than it is for them - they still have STVL
and their own inflation. A softer version for us is to keep the layer enabled
and drive to the remembered free pose with the goal-heal disc pointed at it.

## A2. Nav2 gets thirty retries here and three there

**(a)** Ours:
`rover/src/navi_nav2/behavior_trees/navigate_to_pose_no_reverse.xml:19` sets
`number_of_retries="30"` on the outer `RecoveryNode`, and the recovery is a
`RoundRobin` of Clear, `Spin spin_dist="1.57"` and `Wait wait_duration="5"`
(lines 76-83). At `max_rotational_vel: 0.2`
(`rover/src/navi_nav2/params/nav2_rover.yaml:225`) the spin alone is about
eight seconds, so one full round is fifteen to twenty seconds and thirty
retries is ten rounds. The through-poses tree is the same
(`navigate_through_poses_no_reverse.xml:18`, `:42-56`).

Theirs: `kalman_nav2/behavior_trees/nav_to_pose.xml:3` sets
`number_of_retries="3"`, and the recovery is *only* two `ClearEntireCostmap`
calls (lines 15-18). No Spin, no Wait, no BackUp inside Nav2 at all. Their
through-poses tree allows six (`nav_through_poses.xml:3`).

**(b)** Yes, for the "stalls" half of the symptom. Our run reports `running` on
`/nav_status` for minutes while the rover does nothing an operator would call
progress, and only then aborts. Theirs surfaces the failure to the layer that
can actually do something about it within about fifteen seconds.

**(c)** Cheap, but only correct **together with A1**. Cutting the retry budget
without an escalation path above Nav2 makes the current behaviour strictly
worse. The retry count was raised to 30 deliberately (the comment at
`navigate_to_pose_no_reverse.xml:9-18` records the reasoning) precisely because
there was nothing above Nav2 to escalate to.

## A3. Two settings we never wrote down that they set explicitly, both free

**(a)** `wait_for_service_timeout`: ours is `1000`
(`rover/src/navi_nav2/params/nav2_rover.yaml:31`); theirs is `10000` with the
comment `# default: 1000` (`kalman_nav2/config/nav2.yaml.j2:92`), which is a
team writing down that they hit the default and raised it tenfold.
`bt_navigator` uses this budget when a BT node first waits for its action
server; a tree that cannot bind `FollowPath`, `Spin`, `Wait` or
`ClearEntireCostmap` fails the goal.

`action_server_result_timeout`: we set it nowhere in
`rover/src/navi_nav2/params/nav2_rover.yaml`. They set it on four servers -
`behavior_server` 900.0 (`nav2.yaml.j2:38`), `bt_navigator` 900.0 (`:94`),
`planner_server` 900.0 (`:158`) and `controller_server` **86400.0** with the
comment `# 24h` (`:230`). The 24 h on the controller is the tell: a `FollowPath`
goal that outlives the server's result timeout has its goal handle expire and
comes back as a failure with no useful reason attached.

**(b)** Yes for both, and both match "the run terminated and the log does not
say why".

For `wait_for_service_timeout`, our own bring-up documents the exact condition
that eats a one second budget:
`rover/src/navi_localization/launch/localization.launch.py:33-43` measures ROS 2
discovery stalling about three seconds per ten publishers when a foreign
participant shares the domain, and the ZED wrapper alone makes roughly a
hundred. The first Go after a bring-up is the run at risk.

For `action_server_result_timeout`, our own numbers make 900 s reachable:
`desired_linear_vel: 0.2` (`nav2_rover.yaml:166`) means 900 s is 180 m of
driving, but a single `FollowPath` goal also spans the whole recovery churn of
A2, and thirty rounds of Clear/Spin/Wait is comfortably fifteen minutes.

**(c)** Free. Six lines of YAML. Do this first, before anything structural.

## A4. Nobody on our side checks the goal against a costmap; theirs does, every time

**(a)** Ours: `goal_relay.py:570-600` dispatches `SEND_GOAL` with the
operator's x and y unchanged, and `nav2_goals.py:102-138` builds the
`PoseStamped` from them. The only goal repair in the stack is indirect and
upstream: `traversability_layer.py:601-609` forces a 1.4 m disc around the
active goal free **in the seed**, and
`rover/src/navi_nav2/params/nav2_rover.yaml:85` lets Smac shift an obstructed
goal by up to 1.0 m. Neither is a check; the first edits the map so the goal
looks reachable, the second is a fallback inside the planner.

Theirs: `kalman_supervisor/kalman_supervisor/modules/nav.py:114-131` runs
before every goal is sent. An out-of-bounds goal is clamped toward the map
centre (`modules/map.py:308-332`). A goal that is not `FREE` is moved to the
nearest free or partially-occupied cell by an outward spiral search bounded at
10000 samples (`modules/map.py:207-260`). Only then does the goal go to Nav2,
and the corrected pose is remembered (`nav.py:131`, read back at
`nav.py:476-479`). The supervisor can do this because
`kalman_supervisor/launch/supervisor.launch.py:56-59` remaps its `map/map` and
`map/map_updates` onto `global_costmap/costmap` and its updates - the supervisor
reads the same costmap Nav2 plans on.

**(b)** Yes. Our approach has three weaknesses theirs does not. The disc edits
the seed, so the inflation layer can put cost straight back into it from lethal
cells just outside the disc. The disc is applied at seed rate (about 1 Hz, see
A6) while the goal is sent immediately, so the first plan attempt can run
against an unhealed map. And the coarse seed the global planner actually reads
re-prices unknown separately afterwards
(`traversability_layer.py:638-641`), so what the planner sees near the goal is
not exactly what was healed.

**(c)** Medium, and right for us. It is one subscription and one spiral search;
we already have the geometry helpers (`traversability.py` `_disc_offsets`).
Moving the goal is also more honest than erasing measured lethals, which is
what `heal_goal_patch` does today.

## A5. A leg longer than about 23 m cannot be planned at all, and we have no partial goals

**(a)** Ours: the global costmap is a 48 m rolling window centred on
`base_footprint` (`rover/src/navi_nav2/params/nav2_rover.yaml:318-320`), so a
goal more than about 23.5 m away is off the costmap and both planners reject it
outright. Nothing clamps or splits: `nav_run.py:235-239` sends the raw waypoint
and `goal_relay.py:570-600` passes it through.

Theirs: the same rolling constraint exists (50 m at 0.2 m,
`kalman_nav2/config/nav2.yaml.j2:8-10`) and is solved explicitly.
`NavWithLongGoals` (`kalman_supervisor/kalman_supervisor/modules/nav.py:436-549`)
marks a goal `OUT_OF_BOUNDS` as "long" (`nav.py:536-537`), lets `BasicNav` clamp
it to the map edge and drive there, then re-sends the real goal every time the
robot is within 3 m of the partial goal (`nav.py:469-490`) or the real goal
enters the map (`nav.py:452-467`) - with a 2 m "did we actually advance" guard
(`nav.py:482-484`, `nav.py:497-514`) so a stuck rover does not loop forever.

**(b)** Conditionally, and it is latent rather than proven. The evidence in our
own repo suggests current legs are short - `nav_run.py`'s header speaks of a
1.5 m shortest leg and the 2026-09-01 ride diary quoted in
`nav2_rover.yaml:150-153` mentions 11.30 m - so this may not be biting today.
It will bite at ERC distances, and when it does the symptom is exactly "the
operator's waypoint list is correct, the plan is drawn on the ground station,
Nav2 refuses".

**(c)** Medium. Their design is directly transplantable and needs the same
costmap subscription A4 needs. Nothing about it is wrong for a smaller, slower
rover; if anything a slower rover benefits more, because it spends longer inside
each partial leg.

## A6. Their controller cannot fail; ours can, after half a second, against a map with no live input

**(a)** Ours: `rover/src/navi_nav2/params/nav2_rover.yaml:112` sets
`failure_tolerance: 0.5`. The controller is `RotationShimController` over
`RegulatedPurePursuitController` with `use_collision_detection: true` and
`max_allowed_time_to_collision_up_to_carrot: 2.0` (`nav2_rover.yaml:186-187`),
so RPP raises a "collision ahead" exception whenever the carrot path crosses a
cell the local costmap calls lethal. After 0.5 s of that, `FollowPath` aborts,
which costs one of the thirty retries and a full recovery round.

The map it fails against has no live sensor input at all. The global
`obstacle_layer` is `enabled: false` (`nav2_rover.yaml:347`), the local
`voxel_layer` is `enabled: false` (`:414`), and the collision monitor's
`points_filtered` source is `enabled: false` (`:297`) - all three waiting on a
`cloud_filter` that does not exist, which
`rover/src/navi_nav2/launch/nav2_bringup.launch.py:37-40` records as a MUST DO.
The only obstacle evidence is the latched seed, and its end-to-end latency is
around two seconds: the ZED fused cloud is 1.0 Hz
(`rover/src/navi_localization/launch/localization.launch.py:7-9`), the tile
aggregator publishes at 1.0 s (`tile_aggregator.py:80`), and the derive itself
is about 300 ms at 960 x 960 (`traversability_layer.py:10-11`).

Theirs: `kalman_nav2/config/nav2.yaml.j2:239` sets
`failure_tolerance: -1.0 # infinite`, and the controller is a service that
structurally cannot fail. `kalman_nav2/kalman_nav2/path_follower_node.py:246-465`
always returns a `ComputeVelocityCommands` response: an empty path returns a
zero twist (lines 284-285), a failed tf lookup returns a zero twist and logs
(lines 279-281), and there is no collision check anywhere in the file. Their
obstacle picture is fed at camera rate from four D455s through
`kalman_clouds` (voxel grid leaf 0.07 m, `kalman_clouds/config/voxel_grid.yaml:6`;
radius outlier removal 0.3 m / 20 neighbours,
`kalman_clouds/config/radius_outlier_removal.yaml:6-8`), then
`kalman_nav2/config/obstacle_detection.yaml`, then two STVL layers with
`clear_after_reading: true` and `decay_acceleration` doing active clearing
(`kalman_nav2/config/nav2.costmap.yaml.j2:105-148`).

**(b)** Yes. This is the mechanism that turns one phantom lethal in a 1 Hz map
into an aborted `FollowPath`, and with no contradicting sensor arriving for a
whole second the phantom is authoritative for that whole second. It is also why
the seed needed four hand-built amnesty mechanisms (startup patch, wheel trail,
rover heal, goal heal) where their phantoms simply decay.

**(c)** The `failure_tolerance` change is free but must not be made alone.
Raising it to several seconds or to -1.0 removes RPP's collision check as a
practical guard, and today the collision monitor that would replace it is
disabled (`nav2_rover.yaml:297`). Do the two together: turn the monitor source
on when `cloud_filter` lands, and raise `failure_tolerance` in the same change.
The perception half is expensive and already scheduled; it deserves to be ahead
of further seed tuning.

## A7. The rotation hand-off is throttled by the shaper; theirs is deferred below the controller

**(a)** Ours: `rover/src/navi_shaper/navi_shaper/shaper.py:44-52` gives a
steering change in the 1.20 rad bucket - which the comment at line 48 identifies
as "straight <-> point turn lives here" - a hold of 60 IK ticks, 3.36 s, and
`shaper.py:75` sets `min_gain: float = 0.05`. During that window the whole
twist is scaled, recovering linearly to 1.0 as the window closes. That window
opens at exactly the `RotationShimController` hand-off
(`rover/src/navi_nav2/params/nav2_rover.yaml:137-143`), in both directions.

Theirs: the realignment is handled by the wheels node, below the controller and
invisible to it. `kalman_wheels/kalman_wheels/twist_controller_node.py:165-210`
enters `AdjustWheelsState` when any wheel needs more than
`max_wheel_turn_diff: 0.6` rad of swivel
(`kalman_wheels/config/twist_controller.yaml:18`), holds the drive until every
wheel is inside `min_wheel_turn_diff: 0.2` rad (`:20`), and only then returns to
`DriveState`. The follower's command is never scaled; it is deferred and then
executed in full.

**(b)** Yes, for the "correct plan, nothing visibly happens for several
seconds" reports, and it compounds with the progress checker.
`nav2_rover.yaml:121-130` asks for 0.25 m or 0.20 rad within 60 s; a 0.5 rad/s
command scaled to `min_gain` is 0.025 rad/s, which needs eight seconds to make
0.2 rad even assuming the chassis executes a command that small, and the
comment at `nav2_rover.yaml:158-163` already records the chassis refusing
0.05 rad/s as below its deadband.

**(c)** Medium, and partly wrong-for-us as a straight copy. Our shaper exists
because the real 2.42 IK genuinely sweeps the ICR through the wheelbase, and
their four-wheel swivel model is simpler. The adoptable part is the *placement
and the shape*, not the policy: hold-then-release (zero the drive, let the
steering slew at its own rate, restore full gain) matches what the chassis
actually does better than scale-everything-down, and it does not present the
progress checker with motion that is technically nonzero and practically nil.

## A8. Nobody sees a dead node

**(a)** Ours: a grep across `rover/src` finds no health, watchdog or liveness
monitor of any kind. `rover/start_navi.sh:498-505` starts `nav2_bringup`,
`goal_relay` and `glare_watch` as bare background processes; the only
post-start check in the script is the localisation gate
(`start_navi.sh:235-294`), which runs once. No node has `respawn=True`
(`rover/src/navi_nav2/launch/nav2_bringup.launch.py:142-151`); the only
resilience is the lifecycle manager's bond
(`nav2_rover.yaml:447-449`), which notices a node that crashes its lifecycle,
not a process that dies. A dead `goal_relay` means `/nav_status` simply stops,
and the operator's Go button does nothing forever.

Theirs: `kalman_health/kalman_health/topic_health_monitor_node.py` subscribes to
every configured topic with a per-topic timeout
(`kalman_health/config/monitors.yaml`: IMU 1 s, GPS 10 s, master 5 s, each of
the four D455 colour streams 1 s) and publishes a packed status byte at 3 Hz,
which goes to the master and the LEDs. Separately,
`kalman_nav2/launch/navigation_launch.launch.py:116-200` plumbs `use_respawn`
with a 2 s delay through every Nav2 node.

**(b)** Partly. It does not cause a stall, but it turns every stall into an
unexplained one, and it is why the failures under investigation are described
by symptom rather than by cause.

**(c)** Cheap. Their monitor node is 76 lines and is generic - a topic name, a
type string and a timeout per entry. Ours would want `/localization/pose`,
`/autonomy/map`, `/autonomy/costmap_seed`, `/autonomy_twist`, `/nav_status` and
`/chassis_twist`, published as JSON on the convention the rest of the stack
already uses. Adding `respawn=True` to the Nav2 nodes is one keyword.

---

# B. Worth adopting, but not failure-explaining

## B1. We replan half as often on a grid four times as large

Ours: `navigate_to_pose_no_reverse.xml:21` replans at `hz="1.0"` and
`nav2_rover.yaml:57` sets `expected_planner_frequency: 1.0`, against a global
costmap of 48 m at 0.10 m, which is 480 x 480 = 230,400 cells
(`nav2_rover.yaml:315-320`). Theirs: `nav_to_pose.xml:5` replans at `hz="2.0"`
with `expected_planner_frequency: 10.0` (`nav2.yaml.j2:156`), against 50 m at
0.2 m, which is 250 x 250 = 62,500 cells (`nav2.yaml.j2:8-10`). If replan
latency turns out to be part of a stall, coarsening the global costmap to 0.2 m
is a lever they have already pulled and our seed already supports (the coarse
seed publisher at `traversability_layer.py:642` would simply need one more
halving, and `coarsen_cost` is max-pooled so it can only err toward caution).

## B2. No path smoother

They run a `smoother_server` (`nav2.yaml.j2:203-220`) and the BT smooths before
following (`nav_to_pose.xml:6-11`, `FollowPath path="{smooth_path}"`). We have
no smoother node at all (`rover/src/navi_nav2/navi_nav2/bringup.py:9-16`) and
RPP follows the raw Theta* polyline (`navigate_to_pose_no_reverse.xml:35-45`).
With an any-angle planner the corners arrive as discontinuities, which is
exactly the input a curvature-continuous controller is worst at, and it is also
what opens the shaper's hold windows in A7. Adding `nav2_smoother` is one node
and two BT lines.

## B3. Footprint clearing happens in our seed, at seed rate; in theirs it is a costmap layer

They run an `ObstacleLayer` with no observation sources at all
(`nav2.costmap.yaml.j2:80-83`), whose sole purpose is
`footprint_clearing_enabled: true`. It runs at costmap rate, 5 Hz local and
2 Hz global. Our equivalent is `rover_heal_radius_m` in the seed
(`traversability_layer.py:594-600`), which runs at map rate, about 1 Hz, and is
upstream of Nav2 rather than inside it. Same outcome, one fifth the rate, and
one more thing that stops if the perception chain hiccups. Cheap to add on our
side too, since the layer is already declared and merely disabled.

## B4. Their costmap parameters are written once; ours are written twice

`kalman_nav2/launch/nav2.launch.py:49-89` renders one shared
`nav2.costmap.yaml.j2` and merges it into both the local and global costmap
parameter trees, so footprint, padding, inflation and the OccupancyGrid
translation table cannot drift apart. Ours writes the whole block twice
(`nav2_rover.yaml:299-369` and `:371-441`), including two copies of the
footprint, two of `trinary_costmap`/`lethal_cost_threshold`/`unknown_cost_value`
and two of the inflation settings. Our parameter-fidelity test mitigates the
risk; the duplication is still a standing invitation to change one and not the
other.

## B5. Their inflation has a gradient; ours is nearly binary

Theirs: `inflation_radius: 0.7`, `cost_scaling_factor: 5.0`
(`nav2.costmap.yaml.j2:198-200`). Ours: `inflation_radius: 0.75`,
`cost_scaling_factor: 10.0` on both costmaps (`nav2_rover.yaml:366-367` and
`:438-439`). At a factor of 10 the cost falls by a factor of e every 10 cm past
the inscribed radius, so by 30 cm out it is 5 percent of the peak. That leaves
`use_cost_regulated_linear_velocity_scaling: true` (`nav2_rover.yaml:189`) with
almost no gradient to regulate against, and the planner with almost no reason
to prefer the middle of a gap. Halving the factor to 5.0 would give both
something to work with; note that `inflation_cost_scaling_factor` on RPP
(`nav2_rover.yaml:200`) must move with it, as its own comment says.

## B6. Our slope ceiling's justification cites their default, not their setting

`rover/src/navi_autonomy/navi_autonomy/traversability.py:87-97` states that
30 degrees "is also exactly the ground/obstacle boundary the reference stack
that keeps winning ERC uses (kalman_robot, max_ground_angle 0.7 rad)". 0.7 rad
is 40.1 degrees. The 30 degrees appears in
`kalman_nav2/config/obstacle_detection.yaml:7` only as the node's documented
*default*; the value AGH actually run is 0.7 rad, that is 40 degrees. This does
not make 30 wrong for our rover - the operator picked it from live runs, which
is a better reason than parity - but the comment claims an agreement that is
not there and should be corrected. What genuinely does agree is the fit radius:
their `normal_estimation_radius: 0.2` (`obstacle_detection.yaml:6`) is our
`SLOPE_FIT_RADIUS_M = 0.2` (`traversability.py:178`).

This is a correction to one of the five already-adopted lessons, not a
re-derivation of it.

## B7. They do not model negative obstacles at all

`kalman_nav2/config/obstacle_detection.yaml:13-14` only considers points between
0.5 m below and 0.25 m above the robot, so on their side a hole is an absence of
points and nothing more. Ours condemns any cell 0.14 m below the rover's own
ground (`traversability.py:80`, applied at `traversability.py:474` and
`traversability.py:552-566`). Ours is the safer policy for a yard with a pit and
should stay. The consequence worth holding in mind when reading "the map walled
the rover in" is that our map can produce a whole class of lethal cells theirs
structurally cannot, and every one of them is one plane-fit noise event away
from being a phantom.

## B8. Their static map is never a wall

`kalman_nav2/config/nav2.costmap.yaml.j2:43` sets
`lethal_cost_threshold: 1000000`, which means no value arriving on the map topic
is ever translated to `LETHAL_OBSTACLE`; everything lands in the scaled band and
only STVL can produce a true wall. Ours is 100
(`nav2_rover.yaml:336` and `:399`), so our seed's 100 is a hard stop for the
planner. Deliberate on both sides and probably right for us, since our seed is
the only obstacle source we have. Worth recording because it explains why their
map can be wrong without ending a run, and ours cannot.

## B9. Around the deferred EKF: three things the plan does not cover

The EKF itself is deferred and its plan is
`docs/superpowers/plans/2026-09-02-wheel-odometry.md`. These three are about
what surrounds it.

*Covariance is carried and never consumed.*
`rover/src/navi_localization/navi_localization/localization_status.py:83-84`
stores the ZED's 36-element pose covariance and `:133` puts it on
`/localization/pose`, but nothing subscribes to it as covariance: the only
consumer of localisation health is `supervisor_state.py:132-175`, which gates on
the OK / SEARCHING / OFF string. Their stack consumes covariance structurally
in both EKFs (`kalman_slam/config/ekf_filter_node_local.yaml.j2` and
`ekf_filter_node_global.yaml.j2`), and `gps_preprocessor`
(`kalman_slam/launch/slam.launch.py:127-136`) exists specifically because
`navsat_transform_node` refuses messages whose covariance is not usable.

*The tf tree has one owner.*
`rover/src/navi_localization/launch/localization.launch.py:180-187` gives the
ZED wrapper both `publish_tf` and `publish_map_tf`, so a single process owns
`map -> odom` and `odom -> base_link`; when the camera goes, the whole tree
goes. Theirs splits ownership by design: the local EKF owns `odom -> base_link`
(`ekf_filter_node_local.yaml.j2:69-72`, `world_frame: odom`, `publish_tf: true`)
and the global EKF owns `map -> odom`
(`ekf_filter_node_global.yaml.j2:16-19`, `world_frame: map`). That split is what
makes losing one sensor a degradation rather than an outage, and it is worth
naming as the *reason* the EKF matters, separately from the fusion itself.

*The interim voice is already published and nobody listens.*
`rover/src/navi_localization/navi_localization/twist_odometry.py:170` publishes
`/odom/twist`, and a grep across `rover/src`, `sim` and `ground_station` finds no
subscriber. That is expected while the EKF is deferred; it is worth a line in
the plan so it does not read as dead code later.

## B10. Their supervisor can retune Nav2 at runtime and does so automatically

`modules/map.py:76-109` drives `SetParameters` against both costmaps'
parameter services, and `states/search.py:66-75` does the same to
`path_follower`'s `approach_distance` when the mission changes character. We
have a live retune channel of our own and it is a good one
(`traversability_layer.py:_on_tuning` and `_on_set_parameters`, with ceilings at
`traversability_layer.py:357-371`), but it reaches only the seed producer, never
Nav2's own layers, and nothing in the stack uses it automatically - every retune
is an operator typing a number. The adoptable idea is the automatic use, not
the channel.

---

# C. Differences that are fine

- **Reverse.** We forbid it everywhere
  (`nav2_rover.yaml:9-12`, `:194`, `:214`, and both trees); they drive
  backwards freely (`path_follower_node.py:404-408`, `driving_mode: hybrid`)
  because they have four cameras. Correct for our sensor suite, and the escape
  route A1 offers does not require reversing.
- **Goal yaw.** Ours 6.30 rad (`nav2_rover.yaml:135`), theirs 4.0 rad
  (`nav2.yaml.j2:253`). Both mean "any heading".
- **Progress checking.** Ours is the more forgiving of the two: 0.25 m or
  0.20 rad in 60 s with `PoseProgressChecker` (`nav2_rover.yaml:121-130`)
  against their 1.0 m in 30 s with `SimpleProgressChecker`
  (`nav2.yaml.j2:243-246`). Rotation-aware is the right choice for a rover that
  legitimately spends time turning in place.
- **Velocity arbitration.** Ours is better and must not be traded away. One
  writer on `/rover_twist` with a latched e-stop and two deadmen in series
  (`mode_supervisor.py:16-19`, `supervisor_state.py:22-23`), a feasibility clamp,
  and a collision monitor with the last word on every velocity including the
  recovery behaviours' (`nav2_bringup.launch.py:18-35`). Theirs has no arbiter:
  the velocity smoother publishes straight onto `/cmd_vel`
  (`navigation_launch.launch.py`, remap `cmd_vel_smoothed` to `cmd_vel`), the
  gamepad path reaches the same wheels node, and only the state machine keeps
  them apart. Their wheels node's own 1.0 s `stop_timeout`
  (`kalman_wheels/config/twist_controller.yaml:6`) is their whole deadman.
- **Operator telemetry.** Ours is richer for a human: `/nav_status`,
  `/mode_status`, `/ik_feasibility`, `/autonomy/tuning_state` and the run diary
  (`run_log.py`), all JSON over rosbridge. Theirs is a status byte, an LED
  (`modules/ueuos.py`) and RViz, plus a GPS-space plan for the ground station
  (`kalman_nav2/kalman_nav2/geo_path_converter_node.py`). Different ground
  stations, both adequate.
- **Waypoint handshake.** Our run pauses at every waypoint and needs an operator
  Resume (`nav_run.py:250-263`) because the primary's coordinator moves to
  Waiting; theirs chains automatically (`states/travel.py:32-44`). Imposed by
  the coordinator, not chosen, and changing it means changing the primary.
- **Planner choice.** Theta* with a Smac fallback
  (`nav2_rover.yaml:58-100`) against their NavFn A* (`nav2.yaml.j2:161-167`).
  Any-angle is the right call for a chassis that pays real time for every ICR
  change, and the two-planner fallback has no equivalent on their side.
- **Fault tolerance inside the supervisor.** Their supervisor try/excepts every
  module tick and continues (`supervisor_node.py:117-149`); ours does the same
  per callback in `goal_relay.py`, `mode_supervisor.py` and
  `traversability_layer.py:512-519`. Parity.

---

# Suggested order of work

1. The two free parameters in A3. Six lines, no risk, and both match the
   symptom.
2. The health monitor and `respawn=True` from A8, so the next failure is
   diagnosable rather than described.
3. A1 and A2 together: a recovery ladder above Nav2 (remembered free pose,
   re-send the original goal), and only then cut the retry budget.
4. A4 and A5 together, since both need the same costmap subscription in
   `goal_relay`.
5. A6's `failure_tolerance` change, at the same time as enabling the collision
   monitor source when `cloud_filter` lands.
6. A7's shaper reshaping, and B2's smoother, which attack the same corner.
