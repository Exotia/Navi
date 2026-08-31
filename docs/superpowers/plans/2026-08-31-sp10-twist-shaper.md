# SP10: Twist Shaper — Feasibility Clamp on the Real 2.42 IK

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A node sits between `mode_supervisor` and `bema_bridge` and guarantees that every twist reaching the chassis is one the real inverse kinematics can actually execute. The rover's IK cannot jump between turn geometries: moving the instantaneous centre of rotation from "straight ahead" to "turn in place" takes the steering **up to 3.36 s** on the vendored 2.42 model with Asterope geometry, and during that sweep the rover follows neither the old command nor the new one. `twist_shaper` detects those geometry changes from the commanded twist alone, scales the command down for the duration of the sweep so the ground covered while the wheels are wrong is small, publishes the result on `/chassis_twist`, and reports what it did on `/ik_feasibility`. `bema_bridge` is re-pointed to `/chassis_twist` by a parameter in `start_navi.sh`. The whole path is then verified in the sim by driving a scripted straight → arc → point-turn sequence through the shaper into the real IK and checking that the executed trajectory tracks the commanded one.

**Architecture:** One new ament_python package `rover/src/navi_shaper`. Two pure modules with no ROS in them — `ik_geometry.py` (the command → ICR → per-wheel steering-angle map, a **bit-exact** transcription of the 2.42 model's arithmetic) and `shaper.py` (the gain policy and its hold-window state machine, driven by an injected clock) — plus the rclpy node `twist_shaper.py`, which subscribes `/rover_twist`, publishes `/chassis_twist` **synchronously in the subscription callback** (no timer, no resampling, one DDS hop of added latency) and `/ik_feasibility` as JSON in a `std_msgs/String` at 2 Hz. The oracle for "feasible" is the real vendored model: a committed C++ harness, `sim/src/navi_sim_ik/test/feasibility_harness_242.cpp`, sweeps the 2.42 model and emits a golden fixture that the Python tests assert against — the exact map to 1e-12, and the hold-time table as measured settle counts. Nothing compiled ships to the Orin.

**Tech Stack:** Python 3.10, rclpy (ROS 2 Humble), `geometry_msgs`, `std_msgs`, colcon (ament_python), pytest. C++17 for the offline harness only (laptop-side, built by hand, never a CMake target).

**Spec:** `docs/superpowers/specs/autonomy-plan.md` — §2 (where the shaper sits in the pipe), §5 (the controller block and the regime the IK never clips), §8 SP10 row, §9 rungs 1–2, §10 (speeds), §11 risk 3.

**Depends on:** SP4 (delivered `sim/src/navi_sim_ik/vendor242/` and `include/navi_sim_ik/asterope_params.hpp`), SP5 (delivered `mode_supervisor` as the single writer of `/rover_twist`). SP9's Nav2 bringup is a *consumer* of this work, not a prerequisite: every task here is verifiable with a scripted twist sequence and needs no planner.

---

## Global Constraints

Verbatim from the brief and the spec, and non-negotiable:

- **Zeros pass through unmodified and immediately.** The shaper must never delay or reshape a stop. A twist whose `linear.x`, `linear.y` and `angular.z` are all exactly `0.0` is republished byte-identically on `/chassis_twist` from an **early return at the top of the callback**, before any shaping math runs. The estop and deadman zero streams from `mode_supervisor` therefore traverse the shaper as a pure relay.
- **The shaper NEVER outputs a twist larger than its input.** Componentwise: `|out.x| <= |in.x|`, `|out.y| <= |in.y|`, `|out.z| <= |in.z|`, and every component keeps its input's sign or is zero. The only shaping operation permitted is multiplication of all three components by a single gain `k ∈ [0, 1]`. This is asserted as a property test over a randomised command set, not merely by inspection.
- **The `/rover_twist` single-writer rule stays intact.** `mode_supervisor` remains the sole publisher of `/rover_twist`; `twist_shaper` only ever *subscribes* to it. The shaper lives in its own package and its own process precisely so that it is structurally incapable of publishing to `/rover_twist`.
- **`bema_bridge` is re-pointed by parameter only.** `rover/start_navi.sh` passes `-p twist_topic:=/chassis_twist`. No change to `bema_bridge.py` — the same mechanism SP5 used in its Task 6. The bridge's own 1 s deadman is untouched and remains the last line of defence: if `twist_shaper` dies, `/chassis_twist` goes silent and the bridge stops the rover within 1 s.
- **Speed caps 0.05 m/s and 0.1 rad/s (§10).** The shaper carries them as a *backstop*, not as their owner — see ruling 2.
- **Pure feasibility math is separated from rclpy.** `ik_geometry.py` and `shaper.py` import nothing from `rclpy` and are tested by plain pytest with no ROS graph (spec §9 rung 1). The node module is the only place `rclpy` appears.
- **Tests never publish to `/manual_twist`.** ROS-graph tests use throwaway `ROS_DOMAIN_ID` 91 / 92 / 93, never domain 0. The sim verification remaps the sim IK node's subscription to `/sim_test_twist`, the pattern SP4 Task 6 established. **Kill nodes with a path pattern under `pgrep -f` / `pkill -f` (e.g. `navi_shaper/twist_shaper`), or with `-x` for compiled executables (e.g. `sim_ik_node`); never a `pkill -f` pattern that matches the shell running it.** `-x` alone is not a workable doctrine here: it matches the kernel's `comm`, which for a setuptools console script is `python3`, so an exact-name kill on `twist_shaper` silently matches nothing. The path form is what `start_navi.sh`'s own `kill_stale` already uses (`pgrep -f "navi_supervisor/mode_supervisor"`) — it matches the full command line and cannot match the invoking shell, so it satisfies this constraint rather than bending it.
- **Commits:** one per task, `git -c user.name="Ole Peters" -c user.email="ole.peters@star-dresden.de" commit`, explicit `git add <paths>`, never `git add -A`, **never push**. On `index.lock`, wait 2 s and retry — other planning and implementation agents work in this tree.
- **Do not touch** `docs/superpowers/plans/2026-08-31-sp8-*`, `-sp9-*`, `-sp11-*`, or any file those sub-projects own. Within `sim/`, this plan **creates one new file** (`sim/src/navi_sim_ik/test/feasibility_harness_242.cpp`) and modifies nothing existing there. `vendor242/` and `asterope_params.hpp` are frozen and must not be edited.
- **Environment:** the Orin is reachable but has no internet and no camera attached; the chassis is not to be driven by any task in this plan. Every verification here is laptop-side — pure pytest plus one headless sim run on domain 91. Nothing in this plan requires the rover to move.

### What the real 2.42 model does — measured, not assumed

Every number below was produced by compiling `sim/src/navi_sim_ik/vendor242/*.cpp` with `navi_sim_ik::kAsteropeHParams`, `kBetaDotMax = 1.5`, `kBetaDdotMax = 250.0`, `kAccelerationFactor = 3.0`, `kIkTimestepSeconds = 0.06`, and driving it closed-loop the way `sim_ik_stepper.cpp` does (feeding `beta_next` / `Beta_dot` back into `beta_hat` / `beta_dot_hat`). Task 2 re-derives all of it into a committed fixture; these are the anchors that fixture must reproduce.

**1. The model self-clamps, and `eta_dot_constrained` is what it delivers.** `ExtY_kinematics_T` carries `input_ICR` (the geometry asked for), `feasable_ICR` (the geometry it will actually steer to this tick), `beta_next[4]` (the steering angles it commands), `omega[4]` (wheel spins) and `eta_dot_constrained[3]` (the body velocity those wheels produce). `sim_ik_stepper.cpp` already integrates the pose from `eta_dot_constrained`. So there is no risk of the IK "clipping" a command in the sense of silently truncating it — it degrades the *geometry* gracefully over several ticks. The shaper's job is therefore **not** to prevent clipping; it is to keep the rover from covering ground while the geometry is in transit.

**2. The command → ICR → steering-angle map is exact and closed-form.** From `kinematics.cpp`:

```
                                      -eta_y                       eta_x
  input_ICR.x = Rmax * tanh( ────────────────────── / Rmax )   .y = tanh( ────────────── ) * Rmax
                              sgn(eta_w)*delta + eta_w                     (same denom)

  with Rmax = 50.0, delta = 0.005  (the '<S1>/Rmax' and '<S1>/delta' constants, lines 826-845)

  beta_next[i] = atan2( ICR.y - hParams[2i+1],  ICR.x - hParams[2i] ) - pi/2     (line 1386)
```

One nuance, since it matters to anyone extending the harness: line 1386 takes that ICR from **`feasable_ICR`**, the geometry the model will actually steer to this tick, not from `input_ICR`, the one asked for. The two coincide **at settle**, which is precisely where both the harness and the fixture sample, so writing the map in terms of `input_ICR` is sound here and the golden test is valid. Sampling mid-transition would not be — there the two differ, and only the optimiser (finding 8) knows `feasable_ICR`.

A Python transcription of exactly these two formulas was checked against the settled model output for twelve commands spanning straight, crab, point turn, combined, reverse and both manual and autonomy speed stages. **Worst error: 0.0 on the ICR and 0.0 on the steering angles (modulo π).** Not "within tolerance" — bit-identical. This is what makes a pure-Python runtime possible.

**3. Steering angles are only defined modulo π.** The model emits `atan2(...) - pi/2 ∈ (-3π/2, π/2]` unwrapped, and its `SteerAngles2SteerSpeed` block explicitly considers `beta` and `beta + π` (lines 1391-1400) because a wheel steered to `β + π` and spun backwards is the same wheel. Concretely, a settled point turn gives `beta_next = [0.7981, -3.9397, -2.3424, -0.7981]`, whose second and third entries are the symmetric values `-0.798` and `+0.798` shifted by exactly `-π`. **Every steering-angle comparison in this plan is modulo π, reduced to `(-π/2, π/2]`.** Comparing them naively yields differences of π that are not motion.

**4. `delta = 0.005` means a straight-line command is never straight.** With `eta_w = 0` the denominator is `+0.005`, so at the 0.05 m/s manual cap a "straight ahead" command produces `ICR = (0, 9.869)` — a 9.87 m radius left arc, wheels toed to `[0.0441, 0.0483, -0.0483, -0.0441]`, and `eta_dot_constrained = (0.0495, 0, 0.00496)`: a real 0.005 rad/s left yaw. The effect is *worse the slower you go*: at 0.45 m/s the ICR is 47.3 m, at 0.025 m/s it is 4.98 m, at 0.005 m/s it is 1.00 m.

**5. Consequence: uniform down-scaling is not ICR-preserving, and for near-straight commands it is actively harmful.** Scaling `(0.05, 0, 0)` by `k = 0.2` moves the ICR from 9.87 m to 2.0 m and the steering from 0.044 rad to 0.221 rad — a straight command turned into a 2 m-radius arc. Scaling is exactly ICR-preserving only for a pure point turn (`vx = vy = 0` maps to `ICR = (0,0)` for every `k > 0`) and approximately so for turning commands (`(0.05, 0, 0.1)` at `k = 0.2` shifts the ICR 0.476 → 0.400 m, a 0.035 rad steering error). **This is why the gain policy needs a fidelity guard (ruling 4) and cannot simply scale whatever it likes.**

**6. Scaling does not slow the geometry change — it only reduces the ground covered.** Commanding a point turn from a settled straight run, the steering settles in **26 ticks at every gain tested** (k = 1.0, 0.5, 0.2, 0.05 all settle at tick 26), while the distance travelled during the sweep falls from 0.056 m to 0.014 m. The transition takes as long as it takes; the gain buys distance, which is exactly the quantity that matters for path following.

**7. The hold-time table.** All 342 transitions between the 19 representative commands were swept — every ordered pair of the 19 in `feasibility_harness_242.cpp`, `19 × 18` — each settled from its origin geometry and run until `beta_next` stopped moving (`< 1e-7`). Bucketed on `Δβ`, the exact closed-form steering distance (modulo π, max over the four wheels) between the two commanded geometries:

| `Δβ` bucket (rad) | transitions | **max settle ticks** | seconds | median |
|---|---|---|---|---|
| `[0.00, 0.09)` | 22 | **3** | 0.18 | 1 |
| `[0.09, 0.30)` | 22 | **5** | 0.30 | 3 |
| `[0.30, 0.70)` | 40 | **10** | 0.60 | 7 |
| `[0.70, 1.20)` | 124 | **56** | 3.36 | 17 |
| `[1.20, 1.58]` | 134 | **51** | 3.06 | 20 |

Counts sum to 342, matching `test_fixture_is_complete`'s `19 * 18` and the fixture the harness emits — the same sweep throughout, so the plan's stated evidence and the generated fixture cannot disagree. Across those 342, `HOLD_TICKS` bounds every measured settle count with **zero violations** and a **mean slack of 28.71 ticks**; that measurement is where `test_hold_table_bounds_every_measured_transition`'s `< 50` threshold comes from, and it is what keeps the table from being a huge constant that would pass the safety assertion while crippling the rover.

Two things to read out of this table. First, it confirms spec §5 quantitatively: below one tick's worth of steering (`β̇max · TS = 1.5 × 0.06 = 0.09 rad`) the chassis settles within 3 ticks, so **RPP's curvature-continuous output — a slowly moving ICR — needs no shaping at all and will run at gain 1.0.** Second, the cost is wildly non-linear: a 0.75 rad change can cost 56 ticks while a 1.45 rad change costs 17, because the price is paid when the ICR must travel *through the wheelbase* (a straight → point-turn transition drags the ICR from 9.87 m to the origin, past all four wheels) or when the model enters `indirect_mode` and swings the ICR out to ±50 m to flip sides. **No smooth function of `Δβ` predicts the tick count.** Hence a bucketed max, taken from the model, and never a fit.

**8. The ICR *dynamics* are not replicable in Python, and the plan does not try.** The Controller block's per-tick ICR rate limit (`cost[i] = (dx²/dy + dy)·β̇max`, `X_tilde[i] = (dy²/dx + dx)·β̇max`, lines 857-874) is only the first stage; it feeds an "Optimal Border Point Calculation" and a "Feasable ICR Optimization" that searches border points and intersections, plus a latched `indirect_mode`. A direct transcription of the rate limit alone was tested against the model's own `feasable_ICR` trajectory and diverged on the very first tick (worst error 3.7 m; 50.4 m on an ICR flip, where the model deliberately routes through `indirect_mode`). This is the finding that settles the oracle question below.

### The feasibility-oracle decision, and why

The brief offered three options: (a) a C++ node, (b) a pybind/ctypes binding, (c) a precomputed table. The answer is **(c), with the table made small and meaningful by first replicating the part that *is* closed-form**:

- **Runtime (pure Python, no compiled artifact on the Orin):** `ik_geometry.py` transcribes the model's static map — `twist → input_ICR → beta_next[4]` — which finding 2 shows is **bit-exact**. Everything the gain policy needs from the command space comes from that map: the geometry a command asks for, and the steering distance between two geometries.
- **The one thing the map cannot give — how long a geometry change takes — comes from a 1-D bucketed table of measured worst-case settle counts (finding 7)**, five constants, keyed on the closed-form `Δβ`. It is a table, but of the *transition cost*, not of the command space.
- **Test-time oracle:** `feasibility_harness_242.cpp` drives the real 2.42 model and emits `ik_feasibility_golden.json`. The Python tests assert the static map against it to 1e-12 and assert that every swept transition's measured settle count is ≤ the bucket the shaper would have used. The harness is not a CMake target — same doctrine as SP4's `golden_harness_242.cpp`: a code generator cannot break the build, cannot install, and cannot accidentally link the 2.41 model.

Rejected, with reasons: **(a) a C++ node** would put `vendor242/` and a C++ toolchain into the rover workspace — the `sim/` and `rover/` workspaces are built separately and the Orin builds only `rover/` — to evaluate arithmetic that is four lines of trigonometry. **(b) a binding** costs the same vendoring plus a compiled `.so` that must be rebuilt for the Orin's architecture with no internet, and buys exactness in the *dynamics* that the shaper does not need, because the shaper's output is a scalar gain and its policy is a conservative bound by construction. **A raw table over the command space** is the option the brief guessed at and it is genuinely infeasible: feasibility depends on the chassis' current geometry as well as the command, making the domain `(current_ICR, commanded_ICR)` — 4-D, and on top of that the model's own state carries a latched `indirect_mode` that a static table cannot represent.

### Rulings on spec ambiguities (binding for this plan)

1. **Spec §5 says RPP's output "is the regime the IK never clips" — is the shaper then pointless?** No, and finding 7 says why precisely: in that regime the shaper's gain is 1.0 and it is a pure relay, which is the *correct* behaviour and now a measured fact rather than an assumption. The shaper exists for the residual §5 itself names — mode switches, point-turn transitions, and the `RotationShimController`'s hand-off at aggressive Theta\* corners, all of which step the commanded geometry by far more than 0.09 rad in one message. **Ruling:** the shaper is built for the residual, its no-op behaviour in the RPP regime is an asserted test (`test_rpp_regime_passes_through_unshaped`), and `/ik_feasibility` reports `gain: 1.0` for the whole of a nominal run.
2. **§10 says the speed cap "lives in one place per path (… the velocity smoother for autonomy)" — may the shaper cap?** The shaper is the last node before the chassis and a clamp-down-only ceiling there cannot violate any other constraint. **Ruling:** the shaper carries `backstop_max_vx = 0.05` and `backstop_max_wz = 0.1` as parameters and clamps down to them, but it is documented and named as a **backstop**, not the owner. The smoother (SP9) and `gamepad_input.py` remain the places the cap is *set*; the shaper is where a misconfigured one is caught. Raising these two numbers follows §10's staged ladder and only on explicit instruction.
3. **`/ik_feasibility` has no type in the spec.** **Ruling:** JSON in a `std_msgs/String` at 2 Hz, following SP5's design decision 2 (`/mode_status`, `/drive_status`, `/localization/status` all do this) so the ground station can read it over rosbridge with no ROS installed and no `ament_cmake` message package. `/chassis_twist` stays `geometry_msgs/Twist`, matching `/rover_twist`.
4. **May the shaper scale a near-straight command down?** Finding 5 says doing so turns it into a tight arc. **Ruling:** every candidate gain is checked against an **ICR-fidelity guard** — `k` is admissible only if the steering error its own ICR shift induces is `≤ icr_fidelity_tol`. The error is monotone decreasing in `k`, so the minimum admissible `k` is found by bisection. **The tolerance defaults to 0.10 rad (5.7° at the wheel)**, chosen against this measured table of the lowest admissible gain:

   | command | tol 0.02 | tol 0.05 | **tol 0.10** | tol 0.20 |
   |---|---|---|---|---|
   | `(0.05, 0, 0)` straight | 0.715 | 0.508 | **0.350** | 0.224 |
   | `(0.05, 0, 0.1)` arc | 0.709 | 0.486 | **0.310** | 0.167 |
   | `(0, 0, 0.1)` point turn | 0.050 | 0.050 | **0.050** | 0.050 |
   | `(0.30, 0, 0.20)` | 0.396 | 0.210 | **0.119** | 0.065 |
   | `(0.45, 0, 0.40)` | 0.324 | 0.162 | **0.089** | 0.050 |

   At 0.02 rad the guard would pin a straight or arc command at gain 0.71 and the shaper could barely act at all; at 0.10 rad it can take them to a third of commanded speed. The trade is honest and worth stating: the guard accepts up to 5.7° of steering error in exchange for a 3× reduction in the ground covered during a transition that is *already* carrying up to 0.85 rad (49°) of steering error. A point turn is exempt at any tolerance — scaling preserves its geometry exactly.

   When the guard is what stopped the shaper going lower, `/ik_feasibility` reports `limited_by: "fidelity"` — an honest statement rather than a silent misbehaviour. The straight-line ICR bias itself is **not** correctable by any shaper obeying the never-larger rule (it would require *increasing* `|wz|`); it is reported and handed to SP12 as yard-tuning input.
5. **Two consequences of the geometry map that look like bugs and are not.** Both are asserted by tests so nobody "fixes" them later.

   **(a) A stop RETAINS the geometry — so a resume is free, and a turn after a stop is not.** It is tempting to read the map and conclude that a zero twist asks for `ICR = (0, 0)`, the point-turn pose, and that every resume from a stop therefore costs a 0.8475 rad sweep. **That is false, and getting it backwards is dangerous.** `kinematics.cpp` has a `'<S1>/Retain Translation'` block at lines 798-824, immediately ahead of the ICR Position Controller:

   ```cpp
   smax = VX_out;  s = VY_out;  rtb_eta_dot_idx_2 = U;
   if ((VX_out == 0.0) && (VY_out == 0.0)) {
     if (U == 0.0) {                                  // a full zero twist
       smax = UnitDelay_DSTATE[0];                    // -> reuse the LAST non-zero command
       s    = UnitDelay_DSTATE[1];
       rtb_eta_dot_idx_2 = UnitDelay_DSTATE[2];
     } else { UnitDelay_DSTATE[0..2] = VX_out, VY_out, U; }
   } else {   UnitDelay_DSTATE[0..2] = VX_out, VY_out, U; }
   ```

   Those three values are what feeds `input_ICR`. On a full zero the model deliberately holds the last commanded geometry rather than steering anywhere — that is the block's entire purpose: the wheels stay put so the next resume is free. Measured on the real model, from a settled straight run:

   ```
   after straight settled     icr=[-0, 9.8687660112452]  beta=[0.04412, 0.04827, -0.04826, -0.04412]
   zero, 1 tick               icr=[-0, 9.8687660112452]  beta=[unchanged]
   zero, 61 ticks (3.6 s)     icr=[-0, 9.8687660112452]  beta=[unchanged]
   zero, 461 ticks            icr=[-0, 9.8687660112452]  beta=[unchanged]
   resume straight after a long stop: settle_tick=0  worst_tick_dbeta=0.000000000
   ```

   The `ICR = (0, 0)` reading is true only for a **fresh** model whose `UnitDelay_DSTATE` is still zero — i.e. exactly once, at power-on, before the first command. **Ruling:** the shaper's zero short-circuit relays the stop untouched and immediately, as the Global Constraints require, but **does not clear `_last_cmd`** — its belief about the chassis geometry must survive a stop exactly as the chassis' geometry does. `reset()` does start `_last_cmd` at zero, modelling the boot pose, so the first command after boot is correctly shaped. Two consequences follow, and the second is the safety one: (i) a resume of the same geometry is a pure relay at gain 1.0, so Nav2's replan pauses, goal boundaries and recoveries cost nothing; (ii) the sequence **settled straight → supervisor zero stream → point-turn command** is still a full-size transition — the model spends 26 ticks and 0.0441 m of wrong-geometry travel on it — and the shaper still opens the full hold. Clearing `_last_cmd` would make `command_distance((0,0,0), (0,0,0.1)) = 0`, open no hold, report `feasible`, and pass a full-speed point turn into the single largest geometry change the chassis can make — on a drive→stop→turn sequence a `RotationShimController` hand-off produces routinely. Note that Task 2's fresh-model sweep cannot see any of this on its own, since it re-`initialize()`s per row; that is why the harness carries a separate `retained` sweep.

   **(b) A curvature sign change is briefly held.** As `wz` crosses zero at speed the ICR sweeps from +9.87 m out through infinity and back to about −6.6 m. A 20 Hz RPP stream stepping `wz` from `+0.0025` to `−0.0025` — the two commands on opposite sides of zero, taking opposite branches of `sgn(eta_w)*delta` — moves the geometry **0.1378 rad** in one message, above the 0.09 rad a tick can deliver. The shaper opens a 6-tick hold (0.1378 lands in the `[0.09, 0.30)` bucket) at gain `0.09 / 0.1378 = 0.653` — **a 35% dip for 0.36 s**. Stated plainly because it is a larger dip than a first reading suggests, and a reader tuning this should see the real number. It is still right rather than a false positive: the model takes 35 ticks to flip an arc. A pair on the *same* side of zero, such as `(0.05, 0, 0.0025) → (0.05, 0, 0)`, is not a crossing at all and measures 0.0252 rad — below threshold, unshaped.
6. **Latency budget.** **Ruling:** the shaper publishes synchronously inside the `/rover_twist` callback — it is a transform, not a resampler, and adds no timer. Added latency is one DDS hop plus ~40 µs of Python (four `atan2` calls, one 20-iteration bisection), budgeted at **< 5 ms**, against a 50 ms supervisor cadence and a 1 s bridge deadman. The node declares no work in any other callback that could block the twist path.
7. **Ordering in `start_navi.sh`.** **Ruling:** the shaper starts *after* `mode_supervisor` and *before* `bema_bridge`, the same "the consumer's source must have a publisher by the time it subscribes" doctrine SP5 applied. A `--no-shaper` flag exists, and it prints a loud warning that the bridge will then have no publisher and the rover cannot be driven — it does not silently fall back to `/rover_twist`, because a silent fallback is exactly the wiring mistake the flag would otherwise cause.

---

## File structure

- Create `rover/src/navi_shaper/package.xml`, `setup.py`, `setup.cfg`, `resource/navi_shaper`, `navi_shaper/__init__.py`
- Create `rover/src/navi_shaper/navi_shaper/ik_geometry.py` — the bit-exact 2.42 static map. No ROS.
- Create `rover/src/navi_shaper/navi_shaper/shaper.py` — the gain policy and hold-window state machine. No ROS, injected clock.
- Create `rover/src/navi_shaper/navi_shaper/twist_shaper.py` — the rclpy node.
- Create `rover/src/navi_shaper/test/test_ik_geometry.py`, `test/test_shaper.py`, `test/test_twist_shaper.py`, `test/test_golden_parity.py`
- Create `rover/src/navi_shaper/test/ik_feasibility_golden.json` — the fixture, generated by the harness in Task 2.
- Create `sim/src/navi_sim_ik/test/feasibility_harness_242.cpp` — the offline oracle. New file; nothing existing under `sim/` is modified.
- Create `sim/test/sp10_path_following.sh` — the headless sim verification (Task 6).
- Modify `rover/start_navi.sh` — start the shaper, re-point the bridge, add `--no-shaper`.
- Modify `rover/test/test_start_navi_gate.sh` — cover the new flag and wiring.

---

## Task 1: `navi_shaper` package and the bit-exact IK geometry

**Files:**
- Create `rover/src/navi_shaper/package.xml`
- Create `rover/src/navi_shaper/setup.py`
- Create `rover/src/navi_shaper/setup.cfg`
- Create `rover/src/navi_shaper/resource/navi_shaper` (empty file)
- Create `rover/src/navi_shaper/navi_shaper/__init__.py` (empty file)
- Create `rover/src/navi_shaper/navi_shaper/ik_geometry.py`
- Test: create `rover/src/navi_shaper/test/test_ik_geometry.py`

### Steps

- [ ] **Write the test first.** `test/test_ik_geometry.py`, asserting against values read off the real model (Task 2 replaces these hand-copied anchors with a full fixture sweep; they are correct as written and were measured, not guessed):

```python
"""The IK geometry map, checked against the real 2.42 model's own output.

Every expected value here came from compiling sim/src/navi_sim_ik/vendor242
with kAsteropeHParams and reading ExtY_kinematics_T. Task 2 widens this into a
generated fixture; these anchors stay because a hand-readable test that fails
tells you more than a 300-row JSON diff.
"""
import math

import pytest

from navi_shaper import ik_geometry as g


def test_hparams_are_widened_float32_not_double():
    # The rover declares these `const float` and IkController widens them into
    # hParams. A double literal here would differ in the 8th decimal and the
    # steering angles would no longer be the rover's arithmetic.
    assert g.HPARAMS[0] == pytest.approx(0.455269992351532, abs=0.0)
    assert g.HPARAMS[1] == pytest.approx(-0.44385001063346863, abs=0.0)
    # wheel3y is 0.44285, not 0.44385 - the suspected upstream typo SP4
    # transcribed deliberately. If this ever equals HPARAMS[3], someone
    # "fixed" it and the sim no longer matches the rover.
    assert g.HPARAMS[5] != g.HPARAMS[3]


@pytest.mark.parametrize("vx,vy,wz,icr_x,icr_y", [
    (0.05, 0.0, 0.0, -0.0, 9.868766),        # "straight" is a 9.87 m arc
    (0.025, 0.0, 0.0, -0.0, 4.983400),       # halving the speed halves it
    (0.005, 0.0, 0.0, -0.0, 0.999867),       # and 1/10 speed gives a 1 m arc
    (0.45, 0.0, 0.0, -0.0, 47.340301),
    (0.05, 0.0, 0.1, -0.0, 0.476176),
    (0.30, 0.0, 0.20, -0.0, 1.462997),
    (0.0, 0.05, 0.0, -9.868766, 0.0),        # pure crab
    (0.0, 0.0, 0.1, -0.0, 0.0),              # point turn: ICR at the origin
    (0.0, 0.0, 0.0, -0.0, 0.0),              # zero maps there too
    (-0.15, 0.0, 0.0, -0.0, -26.852478),     # the reverse floor
    (0.05, 0.02, 0.07, -0.266664, 0.666627),
])
def test_icr_matches_the_model(vx, vy, wz, icr_x, icr_y):
    x, y = g.commanded_icr(vx, vy, wz)
    assert x == pytest.approx(icr_x, abs=1e-6)
    assert y == pytest.approx(icr_y, abs=1e-6)


def test_steering_angles_match_the_model_for_a_straight_command():
    betas = g.steering_angles(*g.commanded_icr(0.05, 0.0, 0.0))
    for got, want in zip(betas, [0.0441, 0.0483, -0.0483, -0.0441]):
        assert got == pytest.approx(want, abs=1e-4)


def test_steering_distance_is_modulo_pi():
    # A settled point turn reports [0.7981, -3.9397, -2.3424, -0.7981]: the
    # middle two are the symmetric values shifted by exactly -pi, because a
    # wheel at beta+pi spinning backwards is the same wheel. Comparing them
    # naively yields a difference of pi that is not motion.
    assert g.angle_distance(0.7981, 0.7981 - math.pi) == pytest.approx(0.0, abs=1e-12)
    assert g.angle_distance(0.0, math.pi / 2) == pytest.approx(math.pi / 2, abs=1e-12)
    assert 0.0 <= g.angle_distance(1.3, -2.9) <= math.pi / 2


def test_steering_distance_between_commands():
    straight = (0.05, 0.0, 0.0)
    point = (0.0, 0.0, 0.1)
    # The wheels swing 0.8475 rad between these two geometries.
    assert g.command_distance(straight, point) == pytest.approx(0.8475, abs=0.001)
    assert g.command_distance(straight, straight) == pytest.approx(0.0, abs=1e-12)
    # An all-zero twist maps to the point-turn ICR, so the map reports the same
    # 0.8475 rad sweep from it. Read this as a statement about the map, and
    # about the BOOT pose only: a freshly initialize()d model has
    # UnitDelay_DSTATE = [0,0,0] and its ICR really is the origin. It is NOT a
    # statement about stopping. kinematics.cpp's '<S1>/Retain Translation'
    # block (lines 798-824) substitutes the last non-zero command whenever the
    # twist is all zeros, so a stop mid-run holds the geometry it had and a
    # resume is free. shaper.py relies on both halves of that.
    assert g.command_distance((0.0, 0.0, 0.0), straight) == pytest.approx(0.8475, abs=0.001)


def test_a_curvature_sign_change_exceeds_one_tick_of_steering():
    # A genuine sign crossing: wz goes from +0.0025 to -0.0025, so the two
    # commands take opposite branches of sgn(eta_w)*delta and the ICR sweeps
    # from +9.87 m out through infinity and back to about -6.6 m. That is
    # 0.1378 rad of steering demanded in one 20 Hz message - more than the
    # 0.09 rad a tick can deliver - even though wz itself moved by 0.005.
    #
    # Both commands must be on OPPOSITE sides of zero. A pair like
    # (0.05, 0, 0.0025) -> (0.05, 0, 0) is not a crossing at all: both have
    # wz >= 0, both take the +delta branch, and the measured distance is
    # 0.0252 rad, comfortably below threshold.
    assert g.command_distance((0.05, 0.0, 0.0025), (0.05, 0.0, -0.0025)) > g.ONE_TICK_BETA
    assert g.command_distance((0.05, 0.0, 0.0025), (0.05, 0.0, -0.0025)) == \
        pytest.approx(0.1378, abs=0.001)
    # Same-side, and not a crossing: below threshold, as the comment above says.
    assert g.command_distance((0.05, 0.0, 0.0025), (0.05, 0.0, 0.0)) < g.ONE_TICK_BETA
    # Away from the crossing, the same sweep rate is comfortably inside it.
    assert g.command_distance((0.05, 0.0, 0.050), (0.05, 0.0, 0.051)) < g.ONE_TICK_BETA


def test_one_tick_of_steering_is_the_feasibility_threshold():
    assert g.ONE_TICK_BETA == pytest.approx(1.5 * 0.06, abs=1e-12)


def test_scaling_a_point_turn_preserves_the_geometry_exactly():
    full = g.commanded_icr(0.0, 0.0, 0.1)
    for k in (0.5, 0.2, 0.05, 0.001):
        assert g.commanded_icr(0.0, 0.0, 0.1 * k) == full


def test_scaling_a_straight_command_wrecks_the_geometry():
    # This is the finding the fidelity guard exists for: scaling 0.05 m/s
    # down to 20% turns a 9.87 m arc into a 2 m arc.
    _, full = g.commanded_icr(0.05, 0.0, 0.0)
    _, fifth = g.commanded_icr(0.01, 0.0, 0.0)
    assert full == pytest.approx(9.8688, abs=1e-3)
    assert fifth == pytest.approx(1.9995, abs=1e-3)
    assert g.command_distance((0.05, 0.0, 0.0), (0.01, 0.0, 0.0)) > 0.15
```

- [ ] Run it, watch it fail with `ModuleNotFoundError` — the honest red.
- [ ] **Create the package scaffold.** `package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://relaxng.org/ns/structure/1.0"?>
<package format="3">
  <name>navi_shaper</name>
  <version>0.1.0</version>
  <description>The Asterope rover's feasibility clamp: shapes /rover_twist into
  a twist the real 2.42 inverse kinematics can execute, and publishes it on
  /chassis_twist for bema_bridge.</description>
  <maintainer email="oxe.pxs@gmail.com">star</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

  `setup.py` mirrors `navi_supervisor`'s exactly, with `package_data` for the fixture Task 2 adds:

```python
from setuptools import setup

package_name = 'navi_shaper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='star',
    maintainer_email='oxe.pxs@gmail.com',
    description="Feasibility clamp between /rover_twist and the chassis.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_shaper = navi_shaper.twist_shaper:main',
        ],
    },
)
```

  `setup.cfg` is the standard two lines (`[develop] script_dir=$base/lib/navi_shaper`, `[install] install_scripts=$base/lib/navi_shaper`).

- [ ] **Write `navi_shaper/ik_geometry.py`:**

```python
"""The 2.42 IK's own command-to-geometry map, transcribed exactly.

This is not a model of the rover's kinematics. It is a transcription of two
formulas out of `sim/src/navi_sim_ik/vendor242/kinematics.cpp`, checked
bit-for-bit against the compiled model's own outputs over the whole command
range this rover uses:

  input_ICR   - the '<S1>/ICR Position Controller' MATLAB Function, lines
                826-845, with the '<S1>/Rmax' and '<S1>/delta' constants.
  beta_next   - the '<S1>/ICR2SteerAngles' MATLAB Function, line 1386.

Nothing here models how the steering *moves* between two geometries. That part
of the model is an optimiser with border-point search and a latched indirect
mode, and it is deliberately not reimplemented - see HOLD_TICKS in shaper.py,
which takes the cost of a transition from measurements of the real model
instead of predicting it.

No ROS. No numpy. Import this from anywhere.
"""
import math
import struct

__all__ = [
    "HPARAMS", "RMAX", "DELTA", "BETA_DOT_MAX", "IK_TIMESTEP_S", "ONE_TICK_BETA",
    "commanded_icr", "steering_angles", "angle_distance", "command_distance",
]


def _f32(value: float) -> float:
    """Widen a float32 to double, the way IkController does.

    The rover declares its wheel offsets `const float` and hands them to
    IkController in a std::vector<float>, whose constructor widens them into
    hParams. Writing 0.45527 as a Python float is a *different number* in the
    8th decimal place, and the steering angles would then not be the rover's
    arithmetic. See the FLOAT, NOT DOUBLE note in asterope_params.hpp.
    """
    return struct.unpack("f", struct.pack("f", value))[0]


# From bemacontroller/src/RoverParameters.h, the #if ASTEROPE block, in the
# order BemaServer.cpp:32 passes them to IkController: wheel1x, wheel1y,
# wheel2x, wheel2y, wheel3x, wheel3y, wheel4x, wheel4y. Wheel order is
# front_left, front_right, rear_right, rear_left.
#
# wheel3y is 0.44285 where the other three use 0.44385 - a 1 mm asymmetry in an
# otherwise symmetric chassis and almost certainly an upstream typo. It is
# transcribed as the rover has it, for the same reason SP4 did: a clamp is only
# trustworthy if it clamps against the model the wheels obey.
HPARAMS = tuple(_f32(v) for v in (
    0.45527, -0.44385,
    0.45527, 0.44385,
    -0.45527, 0.44285,
    -0.45527, -0.44385,
))

#: '<S1>/Rmax' - the ICR is saturated to this radius, so "straight ahead" is
#: represented as a 50 m arc rather than as infinity.
RMAX = 50.0

#: '<S1>/delta' - added to |yaw rate| to keep the ICR division finite. It is
#: also why a straight-line command is not straight: with wz = 0 the effective
#: yaw rate is 0.005 rad/s, which at 0.05 m/s is a 9.87 m radius left arc.
DELTA = 0.005

#: IkController.h. Steering rate ceiling, rad/s.
BETA_DOT_MAX = 1.5

#: IkController.h - the rover hardcodes 0.06 s and its update thread sleeps it.
IK_TIMESTEP_S = 0.06

#: The most steering one IK tick can deliver. A commanded geometry change
#: smaller than this is one the chassis absorbs immediately; larger, and the
#: chassis spends several ticks catching up.
ONE_TICK_BETA = BETA_DOT_MAX * IK_TIMESTEP_S  # 0.09 rad


def commanded_icr(vx: float, vy: float, wz: float) -> tuple:
    """The instantaneous centre of rotation the IK will steer toward.

    Body frame, metres, saturated to +-RMAX. Exactly kinematics.cpp:836-845.
    """
    sign = 1.0 if wz >= 0.0 else -1.0
    denom = sign * DELTA + wz
    return (
        math.tanh(-vy / denom / RMAX) * RMAX,
        math.tanh(vx / denom / RMAX) * RMAX,
    )


def steering_angles(icr_x: float, icr_y: float) -> tuple:
    """The four steering angles that ICR implies. Exactly kinematics.cpp:1386.

    Returned unwrapped, in (-3*pi/2, pi/2], the way the model emits them. Use
    angle_distance() to compare two of these - never plain subtraction.
    """
    return tuple(
        math.atan2(icr_y - HPARAMS[2 * i + 1], icr_x - HPARAMS[2 * i]) - math.pi / 2
        for i in range(4)
    )


def angle_distance(a: float, b: float) -> float:
    """How far a steering axle must actually turn to get from a to b.

    Modulo pi, reduced to [0, pi/2]. A wheel steered to beta + pi and spun
    backwards is the same wheel in the same place, and the model's own
    SteerAngles2SteerSpeed block picks whichever of the two is nearer
    (kinematics.cpp:1391-1400). Plain subtraction reports differences of pi
    that correspond to no motion at all.
    """
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def command_distance(cmd_a, cmd_b) -> float:
    """The largest steering change between two commanded geometries, rad.

    Each argument is (vx, vy, wz). This is the single scalar the gain policy
    keys on: zero when two commands ask for the same geometry (whatever their
    speeds), and at most pi/2.
    """
    betas_a = steering_angles(*commanded_icr(*cmd_a))
    betas_b = steering_angles(*commanded_icr(*cmd_b))
    return max(angle_distance(a, b) for a, b in zip(betas_a, betas_b))
```

- [ ] Run `python3 -m pytest rover/src/navi_shaper/test/test_ik_geometry.py -q` from the repo root with `PYTHONPATH=rover/src/navi_shaper`. All green.
- [ ] Commit: `git add rover/src/navi_shaper` then commit as `SP10 Task 1: navi_shaper package and the bit-exact 2.42 IK geometry map`.

---

## Task 2: The feasibility oracle — C++ harness on the real model, and the golden fixture

The point of this task is that no number in the shaper is invented. The harness compiles the same vendored sources the sim runs, sweeps the command space and every transition between representative commands, and writes what the model did. The Python tests then assert the pure map against it and pin the hold-time table.

**Files:**
- Create `sim/src/navi_sim_ik/test/feasibility_harness_242.cpp`
- Create `rover/src/navi_shaper/test/ik_feasibility_golden.json` (generated, committed)
- Test: create `rover/src/navi_shaper/test/test_golden_parity.py`

### Steps

- [ ] **Write the harness.** `sim/src/navi_sim_ik/test/feasibility_harness_242.cpp`:

```cpp
// Generates rover/src/navi_shaper/test/ik_feasibility_golden.json, the fixture
// SP10's shaper is checked against.
//
// Deliberately NOT a CMake target, for the reasons golden_harness_242.cpp
// gives: it is a code generator, not a test, so keeping it out of the package
// build means it can never break the build, never install, and never
// accidentally link the 2.41 model. Build and run it by hand:
//
//   g++ -std=c++17 -O2 -w \
//     -I/home/ole/star/Navi/sim/src/navi_sim_ik/vendor242 \
//     -I/home/ole/star/Navi/sim/src/navi_sim_ik/include \
//     /home/ole/star/Navi/sim/src/navi_sim_ik/test/feasibility_harness_242.cpp \
//     /home/ole/star/Navi/sim/src/navi_sim_ik/vendor242/*.cpp \
//     -o /tmp/feasibility_harness_242 \
//   && /tmp/feasibility_harness_242 \
//     > /home/ole/star/Navi/rover/src/navi_shaper/test/ik_feasibility_golden.json
//
// It compiles the SAME vendored sources the sim links, with the SAME Asterope
// parameters, so what it prints is the model's own behaviour and not a
// transcription of anything. Regenerate it whenever vendor242/ or
// asterope_params.hpp changes - which, both being frozen, should be never
// without a deliberate re-vendor.

#include <cmath>
#include <cstdio>

#include "kinematics.h"
#include "navi_sim_ik/asterope_params.hpp"

namespace
{

struct Command
{
  double vx;
  double vy;
  double wz;
  const char * name;
};

// Chosen to span everything this rover is asked to do: the manual cap
// (0.05 / 0.1) and each autonomy stage from spec section 10, the wz ceiling,
// the reverse floor from section 5, both crab directions, both turn
// directions, and the three degenerate geometries a clamp meets most often
// (straight, pure crab, pure point turn).
constexpr Command kCommands[] = {
  {0.05, 0.0, 0.0, "straight_manual"},
  {0.05, 0.0, 0.1, "arc_left_manual"},
  {0.05, 0.0, -0.1, "arc_right_manual"},
  {0.0, 0.0, 0.1, "point_turn_left"},
  {0.0, 0.0, -0.1, "point_turn_right"},
  {0.05, 0.0, 0.05, "arc_left_shallow"},
  {0.05, 0.0, -0.05, "arc_right_shallow"},
  {0.02, 0.0, 0.1, "arc_left_tight"},
  {0.15, 0.0, 0.1, "stage2_arc"},
  {0.15, 0.0, 0.0, "stage2_straight"},
  {0.30, 0.0, 0.20, "stage3_arc"},
  {0.45, 0.0, 0.40, "stage4_arc_at_ceiling"},
  {0.45, 0.0, 0.0, "stage4_straight"},
  {0.0, 0.05, 0.0, "crab_left"},
  {0.0, -0.05, 0.0, "crab_right"},
  {0.05, 0.02, 0.07, "combined"},
  {-0.15, 0.0, 0.0, "reverse_floor"},
  {0.02, 0.0, 0.0, "straight_slow"},
  {0.0, 0.0, 0.0, "zero"},
};

constexpr int kCommandCount = sizeof(kCommands) / sizeof(kCommands[0]);

// 400 steps = 24 s. Every transition measured settles well inside that; the
// slowest observed is 56 ticks.
constexpr int kSettleSteps = 400;
constexpr int kTransitionSteps = 300;

// 20 ticks = 1.2 s of an exact-zero twist, the supervisor's stop / estop /
// deadman stream. Long enough that any real steering toward the point turn
// would be plainly visible (the sweep takes 26 ticks and would be a third done
// by now); measured, the drift is exactly 0.0, because Retain Translation
// never lets the zeros reach the ICR controller at all.
constexpr int kZeroHoldSteps = 20;

// beta_next stops changing entirely once the geometry is reached - the model
// is deterministic and has no noise, so this is an exact test, not a
// threshold on a decaying quantity.
constexpr double kSettledEpsilon = 1e-7;

void configure(ExtU_kinematics_T & in)
{
  in.TS = navi_sim_ik::kIkTimestepSeconds;
  in.beta_dot_max = navi_sim_ik::kBetaDotMax;
  in.beta_ddot_max = navi_sim_ik::kBetaDdotMax;
  in.acceleration_factor = navi_sim_ik::kAccelerationFactor;
  for (int i = 0; i < 8; ++i) {
    in.hParams[i] = navi_sim_ik::kAsteropeHParams[i];
  }
}

// Closed loop with the model's own outputs, exactly as SimIkStepper does: a
// kinematic simulation has no encoders, so what the model commanded this tick
// is what it measures next tick.
void feedback(ExtU_kinematics_T & in, const ExtY_kinematics_T & out)
{
  for (int w = 0; w < 4; ++w) {
    in.beta_hat[w] = out.beta_next[w];
    in.beta_dot_hat[w] = out.Beta_dot[w];
  }
}

void drive(kinematics & model, ExtU_kinematics_T & in, const Command & c, int steps)
{
  in.VX_out = c.vx;
  in.VY_out = c.vy;
  in.U = c.wz;   // rad/s, ROS sign - no negation, no degrees
  for (int s = 0; s < steps; ++s) {
    model.setExternalInputs(&in);
    model.step();
    feedback(in, model.getExternalOutputs());
  }
}

}  // namespace

int main()
{
  std::printf("{\n");
  std::printf("  \"_comment\": \"Generated by sim/src/navi_sim_ik/test/"
    "feasibility_harness_242.cpp from the real 2.42 model. Do not hand-edit.\",\n");
  std::printf("  \"ik_timestep_s\": %.17g,\n", navi_sim_ik::kIkTimestepSeconds);
  std::printf("  \"beta_dot_max\": %.17g,\n", navi_sim_ik::kBetaDotMax);

  std::printf("  \"hparams\": [");
  for (int i = 0; i < 8; ++i) {
    std::printf("%s%.17g", i ? ", " : "", static_cast<double>(navi_sim_ik::kAsteropeHParams[i]));
  }
  std::printf("],\n");

  // --- the static map: what geometry each command settles to ---------------
  std::printf("  \"settled\": [\n");
  for (int i = 0; i < kCommandCount; ++i) {
    kinematics model;
    model.initialize();
    ExtU_kinematics_T in{};
    configure(in);
    drive(model, in, kCommands[i], kSettleSteps);
    const ExtY_kinematics_T & o = model.getExternalOutputs();
    std::printf("    {\"name\": \"%s\", \"cmd\": [%.17g, %.17g, %.17g], "
      "\"input_icr\": [%.17g, %.17g], \"beta_next\": [%.17g, %.17g, %.17g, %.17g], "
      "\"eta_dot_constrained\": [%.17g, %.17g, %.17g]}%s\n",
      kCommands[i].name, kCommands[i].vx, kCommands[i].vy, kCommands[i].wz,
      o.input_ICR[0], o.input_ICR[1],
      o.beta_next[0], o.beta_next[1], o.beta_next[2], o.beta_next[3],
      o.eta_dot_constrained[0], o.eta_dot_constrained[1], o.eta_dot_constrained[2],
      (i + 1 < kCommandCount) ? "," : "");
  }
  std::printf("  ],\n");

  // --- the dynamics: what every transition between them costs ---------------
  // This is the half that cannot be computed in Python (the model routes the
  // ICR through an optimiser with a latched indirect mode), so it is measured.
  std::printf("  \"transitions\": [\n");
  bool first = true;
  for (int i = 0; i < kCommandCount; ++i) {
    for (int j = 0; j < kCommandCount; ++j) {
      if (i == j) {
        continue;
      }
      kinematics model;
      model.initialize();
      ExtU_kinematics_T in{};
      configure(in);
      drive(model, in, kCommands[i], kSettleSteps);

      in.VX_out = kCommands[j].vx;
      in.VY_out = kCommands[j].vy;
      in.U = kCommands[j].wz;

      int settle_tick = -1;
      double travel = 0.0;
      double yaw = 0.0;
      for (int s = 0; s < kTransitionSteps; ++s) {
        model.setExternalInputs(&in);
        model.step();
        const ExtY_kinematics_T & o = model.getExternalOutputs();
        double worst = 0.0;
        for (int w = 0; w < 4; ++w) {
          const double d = std::fabs(o.beta_next[w] - in.beta_hat[w]);
          if (d > worst) {
            worst = d;
          }
        }
        feedback(in, o);
        if (settle_tick < 0) {
          // Ground covered while the geometry is still in transit - the
          // quantity the shaper's gain actually reduces.
          travel += std::hypot(o.eta_dot_constrained[0], o.eta_dot_constrained[1]) *
            navi_sim_ik::kIkTimestepSeconds;
          yaw += o.eta_dot_constrained[2] * navi_sim_ik::kIkTimestepSeconds;
          if (worst < kSettledEpsilon) {
            settle_tick = s;
          }
        }
      }
      std::printf("%s    {\"from\": \"%s\", \"to\": \"%s\", \"settle_ticks\": %d, "
        "\"travel_m\": %.17g, \"yaw_rad\": %.17g}",
        first ? "" : ",\n", kCommands[i].name, kCommands[j].name, settle_tick, travel, yaw);
      first = false;
    }
  }
  std::printf("\n  ],\n");

  // --- Retain Translation: what a STOP does to the geometry -----------------
  // The sweep above drives each `from` command on a freshly initialize()d
  // model, which cannot see '<S1>/Retain Translation' (kinematics.cpp:798-824)
  // at all: that block only fires once UnitDelay_DSTATE holds a non-zero
  // command. On a full-zero twist it feeds the ICR Position Controller from
  // the LAST NON-ZERO command instead of from the zeros, so the chassis holds
  // the geometry it had rather than steering to the point turn.
  //
  // This section makes that observable. For each ordered pair it settles
  // `from`, drives kZeroHoldSteps of exact zeros, and only then commands `to`.
  // If Retain Translation behaves as read, these settle_ticks match the
  // fresh-model transition i->j (the stop changed nothing), and NOT the
  // transition zero->j. That is the fixture the shaper's corrected zero policy
  // is answerable to: the shaper must keep believing in `from`'s geometry
  // across the stop, exactly as the model does.
  std::printf("  \"retained\": [\n");
  first = true;
  for (int i = 0; i < kCommandCount; ++i) {
    if (kCommands[i].vx == 0.0 && kCommands[i].vy == 0.0 && kCommands[i].wz == 0.0) {
      continue;   // "zero" as an origin has nothing to retain
    }
    for (int j = 0; j < kCommandCount; ++j) {
      if (i == j) {
        continue;
      }
      kinematics model;
      model.initialize();
      ExtU_kinematics_T in{};
      configure(in);
      drive(model, in, kCommands[i], kSettleSteps);

      // Capture the geometry the stop is supposed to preserve.
      const double icr_before[2] = {
        model.getExternalOutputs().input_ICR[0],
        model.getExternalOutputs().input_ICR[1],
      };

      const Command zero{0.0, 0.0, 0.0, "zero"};
      drive(model, in, zero, kZeroHoldSteps);
      const ExtY_kinematics_T & z = model.getExternalOutputs();
      const double icr_drift = std::hypot(z.input_ICR[0] - icr_before[0],
        z.input_ICR[1] - icr_before[1]);

      in.VX_out = kCommands[j].vx;
      in.VY_out = kCommands[j].vy;
      in.U = kCommands[j].wz;

      int settle_tick = -1;
      double travel = 0.0;
      for (int s = 0; s < kTransitionSteps; ++s) {
        model.setExternalInputs(&in);
        model.step();
        const ExtY_kinematics_T & o = model.getExternalOutputs();
        double worst = 0.0;
        for (int w = 0; w < 4; ++w) {
          const double d = std::fabs(o.beta_next[w] - in.beta_hat[w]);
          if (d > worst) {
            worst = d;
          }
        }
        feedback(in, o);
        if (settle_tick < 0) {
          travel += std::hypot(o.eta_dot_constrained[0], o.eta_dot_constrained[1]) *
            navi_sim_ik::kIkTimestepSeconds;
          if (worst < kSettledEpsilon) {
            settle_tick = s;
          }
        }
      }
      std::printf("%s    {\"from\": \"%s\", \"to\": \"%s\", \"settle_ticks\": %d, "
        "\"travel_m\": %.17g, \"icr_drift_during_stop\": %.17g}",
        first ? "" : ",\n", kCommands[i].name, kCommands[j].name, settle_tick,
        travel, icr_drift);
      first = false;
    }
  }
  std::printf("\n  ]\n}\n");
  return 0;
}
```

- [ ] **Generate the fixture** with the command in the header comment, writing to `rover/src/navi_shaper/test/ik_feasibility_golden.json`. Confirm it is valid JSON (`python3 -m json.tool < …`), has 19 `settled` entries, 342 `transitions` (19 × 18) and 324 `retained` rows (18 non-zero origins × 18 destinations), and that **no `settle_ticks` is `-1`** in either sweep — a `-1` means a transition did not settle inside 300 ticks and the constants below would be wrong. Spot-check one `retained` row by eye: `straight_manual → point_turn_left` must report `icr_drift_during_stop` of exactly `0.0` and roughly the same `settle_ticks` as the fresh-model `straight_manual → point_turn_left` row, **not** the near-zero cost of `zero → point_turn_left`. If it does not, Retain Translation is not behaving as read and Task 3's zero policy must be re-derived before going further.
- [ ] **Write `test/test_golden_parity.py`**, the test that makes the pure map trustworthy:

```python
"""The pure Python geometry map, against the real 2.42 model's own output.

The fixture is generated by sim/src/navi_sim_ik/test/feasibility_harness_242.cpp
from the vendored model. If this test fails, either ik_geometry.py drifted or
someone re-vendored the IK without regenerating the fixture; in both cases the
feasibility clamp is no longer clamping against the model the wheels obey.
"""
import json
import math
import pathlib

import pytest

from navi_shaper import ik_geometry as g
from navi_shaper.shaper import HOLD_TICKS, hold_ticks_for

GOLDEN = json.loads(
    (pathlib.Path(__file__).parent / "ik_feasibility_golden.json").read_text())


def test_fixture_is_complete():
    assert len(GOLDEN["settled"]) == 19
    assert len(GOLDEN["transitions"]) == 19 * 18
    assert len(GOLDEN["retained"]) == 18 * 18
    assert all(t["settle_ticks"] >= 0 for t in GOLDEN["transitions"]), \
        "a transition did not settle within 300 ticks - the hold table is unsafe"
    assert all(t["settle_ticks"] >= 0 for t in GOLDEN["retained"])


def test_constants_match_the_model():
    assert g.IK_TIMESTEP_S == GOLDEN["ik_timestep_s"]
    assert g.BETA_DOT_MAX == GOLDEN["beta_dot_max"]
    assert list(g.HPARAMS) == GOLDEN["hparams"]


@pytest.mark.parametrize("row", GOLDEN["settled"], ids=lambda r: r["name"])
def test_icr_is_bit_exact(row):
    got = g.commanded_icr(*row["cmd"])
    assert got[0] == pytest.approx(row["input_icr"][0], abs=1e-12)
    assert got[1] == pytest.approx(row["input_icr"][1], abs=1e-12)


@pytest.mark.parametrize("row", GOLDEN["settled"], ids=lambda r: r["name"])
def test_steering_angles_are_bit_exact_modulo_pi(row):
    got = g.steering_angles(*g.commanded_icr(*row["cmd"]))
    for mine, theirs in zip(got, row["beta_next"]):
        assert g.angle_distance(mine, theirs) == pytest.approx(0.0, abs=1e-12)


def test_hold_table_bounds_every_measured_transition():
    """The safety property of the whole sub-project.

    For every transition the real model performs, the hold window the shaper
    would open must be at least as long as the model actually takes. An
    under-estimate means the shaper releases the gain while the wheels are
    still sweeping - which is precisely the failure it exists to prevent.
    """
    by_name = {r["name"]: tuple(r["cmd"]) for r in GOLDEN["settled"]}
    worst = []
    for t in GOLDEN["transitions"]:
        d = g.command_distance(by_name[t["from"]], by_name[t["to"]])
        budget = hold_ticks_for(d)
        assert budget >= t["settle_ticks"], (
            f"{t['from']} -> {t['to']}: dBeta={d:.3f} budget={budget} "
            f"but the model took {t['settle_ticks']} ticks")
        worst.append(budget - t["settle_ticks"])
    # Conservative, but not absurdly so: a table that just returned a huge
    # constant would pass the assertion above and cripple the rover. The
    # measured mean slack over all 342 transitions is 28.71 ticks, dominated by
    # the two large-dBeta buckets whose medians (17 and 20) sit far below their
    # maxima (56 and 51). That 28.71 is where the < 50 threshold comes from.
    assert sum(worst) / len(worst) < 50, "the hold table is far more conservative than needed"


def test_a_stop_does_not_move_the_geometry():
    """'<S1>/Retain Translation', kinematics.cpp:798-824.

    A full-zero twist does not reach the ICR Position Controller: the block
    substitutes UnitDelay_DSTATE, the last non-zero command, so the chassis
    holds the geometry it had. This is the model property the shaper's zero
    short-circuit is built on - it relays the stop untouched but keeps its
    belief about where the wheels are.
    """
    assert all(r["icr_drift_during_stop"] == 0.0 for r in GOLDEN["retained"]), \
        "a stop moved the ICR - Retain Translation is not behaving as read"


def test_the_hold_table_bounds_transitions_taken_through_a_stop():
    """The safety property again, on the path the shaper is actually used on.

    Nav2 does not step straight from one command to the next; it stops, then
    turns. The shaper must therefore size its hold from the retained geometry,
    not from the zeros - so the same command_distance(from, to) that bounds the
    fresh-model sweep must also bound the drive-stop-turn sweep.
    """
    by_name = {r["name"]: tuple(r["cmd"]) for r in GOLDEN["settled"]}
    for t in GOLDEN["retained"]:
        d = g.command_distance(by_name[t["from"]], by_name[t["to"]])
        assert hold_ticks_for(d) >= t["settle_ticks"], (
            f"{t['from']} -> stop -> {t['to']}: dBeta={d:.3f} "
            f"budget={hold_ticks_for(d)} but the model took {t['settle_ticks']}")


def test_a_stop_then_a_point_turn_is_a_full_size_transition():
    """The drive-stop-turn sequence of ruling 5(a), pinned as a number.

    Drive straight, stop, turn in place. If the shaper cleared its belief on
    the stop it would compute dBeta = 0 here, open no hold, and pass a
    full-speed point turn into the largest geometry change the chassis can be
    asked to make. The model's own cost says otherwise.
    """
    row = next(r for r in GOLDEN["retained"]
               if r["from"] == "straight_manual" and r["to"] == "point_turn_left")
    assert row["settle_ticks"] >= 20, \
        "a stop did not preserve the straight geometry"
    assert row["travel_m"] == pytest.approx(0.0441, abs=0.005)
    # And the shaper's own view of it, from the pure map, agrees.
    assert g.command_distance((0.05, 0.0, 0.0), (0.0, 0.0, 0.1)) == \
        pytest.approx(0.8475, abs=0.001)


def test_resuming_the_same_command_after_a_stop_costs_nothing():
    """The corollary, and why the corrected policy is cheap as well as safe."""
    for name in ("straight_manual", "arc_left_manual", "point_turn_left"):
        rows = [r for r in GOLDEN["retained"] if r["from"] == name and r["to"] == name]
        assert not rows, "i == j is skipped by construction"
    row = next(r for r in GOLDEN["retained"]
               if r["from"] == "straight_manual" and r["to"] == "stage2_straight")
    # Same geometry family, different speed: the wheels barely move.
    assert row["settle_ticks"] <= HOLD_TICKS[0][1]


def test_the_rpp_regime_needs_no_holding():
    """Spec section 5: a slowly moving ICR is the regime the IK never clips."""
    by_name = {r["name"]: tuple(r["cmd"]) for r in GOLDEN["settled"]}
    small = [t for t in GOLDEN["transitions"]
             if g.command_distance(by_name[t["from"]], by_name[t["to"]]) <= g.ONE_TICK_BETA]
    assert small, "the sweep contains no small-step transitions to check"
    assert max(t["settle_ticks"] for t in small) <= HOLD_TICKS[0][1]


def test_a_straight_command_is_not_straight():
    """Documented so it cannot be silently 'fixed' later.

    delta = 0.005 makes a wz = 0 command read as a 0.005 rad/s left turn, so at
    the 0.05 m/s manual cap the chassis steers a 9.87 m arc and yaws at about
    0.005 rad/s. No shaper obeying the never-larger rule can correct this - it
    would have to increase |wz|. It is reported on /ik_feasibility instead, and
    is SP12 yard-tuning input.
    """
    row = next(r for r in GOLDEN["settled"] if r["name"] == "straight_manual")
    assert row["input_icr"][1] == pytest.approx(9.8688, abs=1e-3)
    assert row["eta_dot_constrained"][2] == pytest.approx(0.00496, abs=1e-4)
    assert row["eta_dot_constrained"][2] != 0.0
```

- [ ] This test imports `shaper.HOLD_TICKS`, which does not exist yet — that is the intended red, and it is what makes Task 3's constants answerable to the model rather than to taste. Run it, see the `ImportError`, and move to Task 3. Do **not** weaken the test to make it pass here.
- [ ] Commit: `git add sim/src/navi_sim_ik/test/feasibility_harness_242.cpp rover/src/navi_shaper/test/ik_feasibility_golden.json rover/src/navi_shaper/test/test_golden_parity.py` then commit as `SP10 Task 2: 2.42 feasibility harness and the golden fixture`.

---

## Task 3: The gain policy

**Files:**
- Create `rover/src/navi_shaper/navi_shaper/shaper.py`
- Test: create `rover/src/navi_shaper/test/test_shaper.py`

### Steps

- [ ] **Write the test first.** `test/test_shaper.py`:

```python
"""The gain policy: pure, clock-injected, no ROS."""
import math

import pytest

from navi_shaper import ik_geometry as g
from navi_shaper.shaper import ShaperConfig, TwistShaper


def make(**kw):
    return TwistShaper(ShaperConfig(**kw))


# --- the safety properties, first ------------------------------------------

def test_zero_passes_through_untouched_and_unheld():
    s = make()
    s.shape(0.05, 0.0, 0.0, dt=0.05)
    s.shape(0.0, 0.0, 0.1, dt=0.05)          # open a hold window
    out = s.shape(0.0, 0.0, 0.0, dt=0.05)    # the estop / deadman stream
    assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)
    assert out.gain == 1.0
    assert out.limited_by == "none"


def test_a_zero_stream_never_becomes_nonzero():
    s = make()
    for _ in range(200):
        out = s.shape(0.0, 0.0, 0.0, dt=0.05)
        assert (out.vx, out.vy, out.wz) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize("vx,vy,wz", [
    (0.05, 0.0, 0.0), (0.0, 0.0, 0.1), (0.05, 0.0, 0.1), (-0.15, 0.0, 0.0),
    (0.0, 0.05, 0.0), (0.45, 0.0, 0.4), (0.02, -0.01, -0.08), (0.3, 0.0, -0.2),
])
def test_output_is_never_larger_than_input_componentwise(vx, vy, wz):
    s = make()
    # Drive a sequence that forces every branch: settled, jumped, recovering.
    for cmd in [(vx, vy, wz), (0.0, 0.0, 0.1), (vx, vy, wz), (0.05, 0.0, 0.0),
                (vx, vy, wz)] * 8:
        out = s.shape(*cmd, dt=0.05)
        assert abs(out.vx) <= abs(cmd[0]) + 1e-15
        assert abs(out.vy) <= abs(cmd[1]) + 1e-15
        assert abs(out.wz) <= abs(cmd[2]) + 1e-15
        assert out.vx * cmd[0] >= 0.0 and out.vy * cmd[1] >= 0.0
        assert out.wz * cmd[2] >= 0.0
        assert 0.0 <= out.gain <= 1.0


def test_the_gain_is_uniform_across_the_three_components():
    """A non-uniform scale would change the geometry, which is the one thing
    the shaper must not do silently."""
    s = make()
    s.shape(0.05, 0.02, 0.07, dt=0.05)
    s.shape(0.0, 0.0, 0.1, dt=0.05)
    out = s.shape(0.05, 0.02, 0.07, dt=0.05)
    assert out.vx == pytest.approx(0.05 * out.gain, rel=1e-12)
    assert out.vy == pytest.approx(0.02 * out.gain, rel=1e-12)
    assert out.wz == pytest.approx(0.07 * out.gain, rel=1e-12)


# --- the policy ------------------------------------------------------------

def settle(s, cmd, seconds=6.0):
    """Hold one command until any hold window has closed.

    The longest hold is 60 ticks = 3.6 s, and the first message after
    construction always opens one - reset() starts _last_cmd at (0,0,0),
    modelling a freshly initialize()d model whose UnitDelay_DSTATE is still
    zero and whose ICR is therefore genuinely the origin. That is a BOOT
    condition, not a stop condition: mid-run, Retain Translation keeps the
    geometry across a zero twist. Either way, 6 s of settling is needed before
    any test that wants a quiescent starting state.
    """
    for _ in range(int(seconds / 0.05)):
        out = s.shape(*cmd, dt=0.05)
    return out


def test_a_settled_command_is_a_pure_relay():
    s = make()
    out = settle(s, (0.05, 0.0, 0.1))
    assert out.gain == 1.0
    assert (out.vx, out.wz) == (0.05, 0.1)


def test_rpp_regime_passes_through_unshaped():
    """A curvature-continuous sweep - the ICR moving a little every message -
    must never be touched. Spec section 5.

    The sweep deliberately stays on one side of wz = 0: crossing zero is a real
    geometry flip and is covered by the next test.
    """
    s = make()
    settle(s, (0.05, 0.0, 0.05))
    for i in range(400):
        wz = 0.05 + 0.04 * math.sin(i / 40.0)
        out = s.shape(0.05, 0.0, wz, dt=0.05)
        assert out.gain == 1.0, f"shaped an RPP-regime command at step {i}"


def test_a_curvature_sign_change_is_briefly_held():
    """Not a false positive: as wz crosses zero the ICR sweeps from +10 m
    through infinity to -7 m, which is more steering than a tick can deliver,
    and the model takes 35 ticks to flip an arc. Worth knowing that ordinary
    driving through a curvature sign change is briefly shaped."""
    s = make()
    settle(s, (0.05, 0.0, 0.02))
    out = s.shape(0.05, 0.0, -0.02, dt=0.05)
    assert out.gain < 1.0
    assert out.delta_beta_rad > g.ONE_TICK_BETA
    assert out.limited_by in ("slew", "fidelity")


def test_a_point_turn_transition_is_held_down():
    s = make()
    settle(s, (0.05, 0.0, 0.0))               # settle straight
    out = s.shape(0.0, 0.0, 0.1, dt=0.05)     # demand a point turn
    # dBeta is 0.8475, so one tick of steering buys 0.09/0.8475 = 0.106 of the
    # commanded motion. A point turn's geometry is scale-invariant, so the
    # fidelity guard does not raise it.
    assert out.gain == pytest.approx(0.106, abs=0.01)
    assert out.limited_by == "slew"
    assert out.wz == pytest.approx(0.1 * out.gain)
    assert out.delta_beta_rad == pytest.approx(0.8475, abs=0.001)


def test_the_hold_releases_and_the_gain_recovers_to_one():
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    gains = [s.shape(0.0, 0.0, 0.1, dt=0.05).gain for _ in range(200)]
    assert gains[0] < 0.2
    assert gains[-1] == 1.0
    assert gains == sorted(gains), "the gain must recover monotonically"
    # 60 ticks at 0.06 s = 3.6 s, the bucket for a 0.8475 rad change, which
    # bounds the model's measured 56-tick worst case.
    held = sum(1 for x in gains if x < 1.0) * 0.05
    assert 3.3 <= held <= 4.2


def test_a_near_straight_command_is_only_partly_scaled():
    """The fidelity guard. The slew policy alone would ask for a gain of 0.106
    here, but scaling (0.05, 0, 0) that far moves its ICR from 9.87 m to 1.2 m -
    a shaper that did that would steer the rover off the very path it is
    protecting. At the default 0.10 rad tolerance the guard floors it at 0.35."""
    s = make()
    settle(s, (0.0, 0.0, 0.1))                # settle a point turn
    out = s.shape(0.05, 0.0, 0.0, dt=0.05)    # now demand straight
    assert out.gain == pytest.approx(0.350, abs=0.01)
    assert out.limited_by == "fidelity"
    assert g.command_distance((0.05, 0.0, 0.0), (out.vx, out.vy, out.wz)) <= 0.10 + 1e-9


@pytest.mark.parametrize("cmd", [
    (0.05, 0.0, 0.0), (0.05, 0.0, 0.1), (0.3, 0.0, 0.2),
    (0.0, 0.05, 0.0), (0.02, 0.0, 0.1), (0.45, 0.0, 0.4),
])
@pytest.mark.parametrize("tol", [0.02, 0.10, 0.20])
def test_the_fidelity_guard_bounds_the_geometry_error_it_allows(cmd, tol):
    s = make(icr_fidelity_tol_rad=tol, backstop_max_vx=0.0, backstop_max_wz=0.0)
    settle(s, (0.0, 0.0, 0.1))    # a point turn, so every cmd below is a jump
    out = s.shape(*cmd, dt=0.05)
    err = g.command_distance(cmd, (out.vx, out.vy, out.wz))
    assert err <= tol + 1e-9, f"{cmd} shaped to a geometry {err:.4f} rad away"


def test_a_stop_retains_the_geometry_so_a_turn_after_it_is_still_held():
    """Ruling 5(a), and the reason the zero path must not clear
    _last_cmd.

    kinematics.cpp's '<S1>/Retain Translation' block feeds the ICR controller
    from the last NON-ZERO command whenever the twist is all zeros, so a stop
    leaves the wheels exactly where they were. Measured: a settled straight run
    held at zero for 461 ticks keeps input_ICR at [-0, 9.8687660112452] with
    beta_next unchanged. The chassis is therefore still in the STRAIGHT pose
    when the point-turn command arrives, and the real model spends 26 ticks and
    0.0441 m getting out of it.

    If the shaper cleared its belief on the stop, command_distance would be 0
    (both a zero twist and a point turn map to the point-turn ICR), no hold
    would open, and a full-speed point turn would go straight through. That
    sequence - drive, stop, turn in place - is what a Nav2
    RotationShimController hand-off produces routinely.
    """
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    for _ in range(40):                       # 2 s of supervisor zero stream
        z = s.shape(0.0, 0.0, 0.0, dt=0.05)
        assert (z.vx, z.vy, z.wz) == (0.0, 0.0, 0.0)
    out = s.shape(0.0, 0.0, 0.1, dt=0.05)
    assert out.delta_beta_rad == pytest.approx(0.8475, abs=0.001), \
        "the stop must not have erased the straight geometry"
    assert out.gain == pytest.approx(0.106, abs=0.01)
    assert out.limited_by == "slew"
    assert out.hold_remaining_s == pytest.approx(60 * g.IK_TIMESTEP_S, abs=0.01)
    assert out.feasible is False


def test_resuming_the_same_command_after_a_stop_is_free():
    """The other half of Retain Translation, and the one that makes the fix
    cheap rather than merely safe.

    Because the wheels did not move during the stop, resuming the SAME geometry
    costs nothing: measured on the real model, settle_tick 0 and 0.000000000
    rad of steering. The shaper must agree, or every Nav2 replan pause, goal
    boundary and recovery would pay a spurious 3.6 s crawl at gain 0.35.
    """
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    for _ in range(40):
        s.shape(0.0, 0.0, 0.0, dt=0.05)
    out = s.shape(0.05, 0.0, 0.0, dt=0.05)
    assert out.delta_beta_rad == pytest.approx(0.0, abs=1e-12)
    assert out.gain == 1.0
    assert out.limited_by == "none"
    assert (out.vx, out.wz) == (0.05, 0.0)


def test_an_overriding_backstop_reports_the_error_it_causes():
    """The backstop is a hard speed ceiling and outranks the fidelity guard -
    a speed cap is a safety limit, geometry fidelity is a quality one. It is
    therefore allowed to push the geometry error past icr_fidelity_tol_rad,
    and when it does, fidelity_err_rad must report the TRUE error rather than
    the tolerance the guard would have enforced on its own."""
    s = make(backstop_max_vx=0.01, icr_fidelity_tol_rad=0.10)
    out = settle(s, (0.05, 0.0, 0.0))
    assert out.limited_by == "backstop"
    assert out.gain == pytest.approx(0.2, abs=0.01)     # 0.01 / 0.05
    assert out.fidelity_err_rad == pytest.approx(0.2365, abs=0.01)
    assert out.fidelity_err_rad > 0.10, \
        "the backstop overrode the guard and the report must say so"
    assert out.fidelity_err_rad == pytest.approx(
        g.command_distance((0.05, 0.0, 0.0), (out.vx, out.vy, out.wz)), abs=1e-12)


def test_backstop_caps_clamp_down_only():
    s = make(backstop_max_vx=0.05, backstop_max_wz=0.1)
    out = settle(s, (0.45, 0.0, 0.4))
    assert abs(out.vx) <= 0.05 + 1e-12
    assert abs(out.wz) <= 0.1 + 1e-12
    assert out.limited_by == "backstop"
    # still a uniform scale of the input, still never larger
    assert abs(out.vx) <= 0.45 and abs(out.wz) <= 0.4
    assert out.vx / 0.45 == pytest.approx(out.wz / 0.4, rel=1e-12)


def test_a_gap_in_the_stream_does_not_blow_up_the_hold():
    s = make()
    settle(s, (0.05, 0.0, 0.0))
    s.shape(0.0, 0.0, 0.1, dt=0.05)
    out = s.shape(0.0, 0.0, 0.1, dt=30.0)   # the link dropped for 30 s
    assert out.gain == 1.0, "a long gap means the chassis has long since settled"


def test_diagnostics_report_the_straight_line_bias():
    s = make()
    out = s.shape(0.05, 0.0, 0.0, dt=0.05)
    assert out.icr[1] == pytest.approx(9.8688, abs=1e-3)
    # 0.05 m/s round a 9.8688 m ICR. The model's own eta_dot_constrained
    # reports 0.00496 rad/s for the same command; the 2% difference is the
    # wheel solution the ICR alone does not capture, and does not matter for a
    # diagnostic whose job is to say "this is not zero".
    assert out.straight_bias_rad_s == pytest.approx(0.00507, abs=1e-4)
    assert out.straight_bias_rad_s > 0.0
```

- [ ] Run it. Red on `ImportError`.
- [ ] **Write `navi_shaper/shaper.py`:**

```python
"""The feasibility clamp's gain policy. Pure: no ROS, no wall clock, no I/O.

The rover's steering cannot jump between turn geometries. Moving the ICR from
"straight ahead" to "turn in place" takes the real 2.42 model up to 56 ticks -
3.36 s - and while that sweep is in progress the chassis follows neither the
old command nor the new one. What this module does about it:

  1. Measure the geometry change each incoming twist demands, as the exact
     closed-form steering distance between the previous commanded geometry and
     this one (ik_geometry.command_distance).
  2. If it is within one tick of steering (0.09 rad), do nothing at all. That
     is the regime a curvature-continuous controller like RPP lives in, and it
     is measurably the regime the IK absorbs immediately.
  3. Otherwise open a hold window whose length is the *measured* worst case for
     a change that size, and scale the whole twist down for its duration, so
     the ground covered while the wheels are wrong is small. The gain then
     recovers linearly to 1.0 as the window closes.
  4. Never choose a gain whose own effect on the geometry exceeds
     icr_fidelity_tol - scaling is not geometry-preserving in this model, and
     for a near-straight command it is actively harmful (see ik_geometry.DELTA).

Everything the policy does is a single uniform gain in [0, 1] applied to all
three components, so the output is never larger than the input in any
component and never points anywhere the input did not.
"""
import math
from dataclasses import dataclass, field

from navi_shaper import ik_geometry as g

#: Worst-case settle time, in IK ticks, as a function of the commanded steering
#: change. Measured on the real 2.42 model over all 342 transitions between the
#: 19 representative commands in feasibility_harness_242.cpp: for each bucket,
#: the largest settle count observed, rounded up to a round number.
#:
#: Do not fit a curve to this. The cost is wildly non-monotone - a 0.75 rad
#: change can cost 56 ticks where a 1.45 rad change costs 17 - because the
#: price is paid when the ICR must travel through the wheelbase, or when the
#: model enters its latched indirect mode and swings the ICR out to +-50 m to
#: flip sides. test_golden_parity.py asserts this table bounds every measured
#: transition; regenerate the fixture and re-check it after any re-vendor.
#:
#: (upper bound of the dBeta bucket, hold ticks)
HOLD_TICKS = (
    (0.09, 3),     # measured max 3   - the RPP regime, effectively a no-op
    (0.30, 6),     # measured max 5
    (0.70, 12),    # measured max 10
    (1.20, 60),    # measured max 56  - straight <-> point turn lives here
    # pi/2, not pi: angle_distance() reduces modulo pi and returns
    # min(d, pi - d), so dBeta can never exceed 1.5708 and a `math.pi` upper
    # bound here would be dead code that reads as if the table covered twice
    # the range it does.
    (math.pi / 2, 56),  # measured max 51
)


def hold_ticks_for(delta_beta: float) -> int:
    """How long the chassis may take to absorb a steering change of this size."""
    for upper, ticks in HOLD_TICKS:
        if delta_beta <= upper:
            return ticks
    # Unreachable for any value angle_distance() can produce (the last bucket
    # ends at its maximum, pi/2). Kept as a defensive floor so a future caller
    # passing an unreduced angle gets the most conservative hold rather than a
    # TypeError on a None return.
    return HOLD_TICKS[-1][1]


@dataclass(frozen=True)
class ShaperConfig:
    #: Never scale below this. Zero would stop the chassis steering toward the
    #: new geometry at all: the IK derives the ICR from the *direction* of the
    #: twist, so any positive gain keeps the wheels sweeping at full rate while
    #: a gain of zero leaves it with nothing to aim at.
    min_gain: float = 0.05
    #: The most steering error the shaper's own down-scaling may introduce.
    #: 0.10 rad is 5.7 degrees at the wheel - see ruling 4 for the measured
    #: table this default was chosen from. Tightening it to 0.02 would floor a
    #: straight command's gain at 0.71 and leave the shaper unable to act.
    icr_fidelity_tol_rad: float = 0.10
    #: Section 10 caps, as a backstop. The smoother and gamepad_input.py own
    #: these numbers; this is where a misconfigured one is caught.
    backstop_max_vx: float = 0.05
    backstop_max_wz: float = 0.1
    #: A gap longer than this means the chassis has settled whatever it was
    #: doing; drop any hold rather than carrying a stale one across a dropout.
    max_dt_s: float = 1.0


@dataclass
class ShapedTwist:
    vx: float
    vy: float
    wz: float
    gain: float
    feasible: bool
    limited_by: str          # "none" | "slew" | "fidelity" | "backstop"
    #: What was limiting *before* the backstop overrode it, so the backstop
    #: does not discard the diagnostic it replaces. "none" unless
    #: limited_by == "backstop".
    also_limited_by: str
    #: command_distance(cmd, output): the geometry error the shaped output
    #: really carries. Bounded by icr_fidelity_tol_rad when the guard set the
    #: gain; unbounded when the backstop overrode it - see shape().
    fidelity_err_rad: float
    delta_beta_rad: float
    hold_remaining_s: float
    icr: tuple = (0.0, 0.0)
    straight_bias_rad_s: float = 0.0


@dataclass
class TwistShaper:
    config: ShaperConfig = field(default_factory=ShaperConfig)

    def __post_init__(self):
        self.reset()

    def reset(self):
        # (0, 0, 0) here is correct and is NOT the same statement the zero
        # short-circuit used to make. It models the model's own zero
        # UnitDelay_DSTATE at initialize(): before a freshly booted kinematics
        # has ever seen a non-zero command, Retain Translation has nothing to
        # retain and the ICR really is the origin, i.e. the point-turn pose. So
        # the very first command after boot is correctly treated as a sweep
        # from the point turn. Only at boot - never again after a stop.
        self._last_cmd = (0.0, 0.0, 0.0)
        self._hold_remaining_s = 0.0
        self._hold_total_s = 0.0
        self._hold_entry_gain = 1.0

    # -- the one entry point ------------------------------------------------
    def shape(self, vx: float, vy: float, wz: float, dt: float) -> ShapedTwist:
        """Shape one commanded twist. `dt` is the time since the previous call."""
        cfg = self.config

        # A stop is relayed untouched and immediately - before any maths runs,
        # so that no future change to the policy below can delay or reshape it.
        #
        # _last_cmd is deliberately NOT cleared here. kinematics.cpp's
        # '<S1>/Retain Translation' block (lines 798-824) feeds the ICR
        # Position Controller from UnitDelay_DSTATE - the last NON-ZERO
        # command - whenever VX, VY and U are all exactly zero. So a full-zero
        # twist does not steer the chassis to the point-turn pose: it RETAINS
        # the geometry it already had. Measured on the real model, a settled
        # straight run held at zero for 461 ticks keeps input_ICR at
        # [-0, 9.8687660112452] with beta_next unchanged, and resuming straight
        # afterwards costs zero ticks and zero radians.
        #
        # The shaper's belief about the chassis geometry must therefore survive
        # a stop exactly as the chassis' own geometry does. Clearing it would
        # make command_distance((0,0,0), point_turn) = 0 - both map to the
        # point-turn ICR - so the sequence
        #   settled straight -> supervisor zero stream -> point-turn command
        # would open no hold, report feasible, and pass a full-speed point turn
        # into a 26-tick / 0.0441 m wrong-geometry sweep. That is a routine
        # Nav2 RotationShimController hand-off, and it is the exact failure
        # this whole module exists to prevent.
        #
        # _hold_remaining_s IS cleared: standing still genuinely does let any
        # in-flight sweep finish, and a stop must not leave a hold armed to
        # bite the next command.
        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            self._hold_remaining_s = 0.0
            return ShapedTwist(0.0, 0.0, 0.0, 1.0, True, "none", "none", 0.0,
                               0.0, 0.0, g.commanded_icr(0.0, 0.0, 0.0), 0.0)

        cmd = (vx, vy, wz)
        icr = g.commanded_icr(*cmd)

        # A gap in the stream means the chassis has had time to settle.
        if dt > cfg.max_dt_s:
            self._hold_remaining_s = 0.0
        else:
            self._hold_remaining_s = max(0.0, self._hold_remaining_s - dt)

        delta = g.command_distance(self._last_cmd, cmd)

        # A demand larger than one tick of steering opens (or re-opens) a hold.
        if delta > g.ONE_TICK_BETA:
            hold_s = hold_ticks_for(delta) * g.IK_TIMESTEP_S
            if hold_s > self._hold_remaining_s:
                self._hold_remaining_s = hold_s
                self._hold_total_s = hold_s
                # What one tick can deliver, over what was asked for: the
                # fraction of the commanded motion the chassis can honour now.
                self._hold_entry_gain = max(cfg.min_gain, g.ONE_TICK_BETA / delta)

        if self._hold_remaining_s > 0.0 and self._hold_total_s > 0.0:
            # Recover linearly as the window closes. The chassis reaches the
            # new geometry at full steering rate regardless of the gain - the
            # gain buys distance, not time - so a linear release is honest.
            progress = 1.0 - (self._hold_remaining_s / self._hold_total_s)
            slew_gain = self._hold_entry_gain + (1.0 - self._hold_entry_gain) * progress
        else:
            slew_gain = 1.0

        fidelity_gain = self._min_faithful_gain(cmd)
        gain = max(slew_gain, fidelity_gain)

        # "fidelity" means the guard is what stopped the shaper going lower -
        # including the case where it stopped it dead at 1.0. "slew" means the
        # hold policy alone set the gain. The distinction is the whole content
        # of the diagnostic: one says "I am protecting the transition", the
        # other says "I wanted to and could not".
        if fidelity_gain > slew_gain:
            limited_by = "fidelity"
        elif gain < 1.0:
            limited_by = "slew"
        else:
            limited_by = "none"

        # The backstop, applied last and downward only. Downward-only means it
        # can never increase a component - but it multiplies `gain` AFTER the
        # fidelity bisection has already chosen the smallest admissible value,
        # so the product is NOT bounded by icr_fidelity_tol_rad and the
        # resulting geometry error can exceed it. That is deliberate, and the
        # precedence is: a speed cap is a SAFETY limit and geometry fidelity is
        # a QUALITY one, so the backstop wins. With backstop_max_vx = 0.01 and
        # a 0.05 m/s command the effective gain is 0.2, whose measured geometry
        # error is 0.2365 rad - 2.4x the 0.10 rad default. The answer is not to
        # weaken the cap but to make the cost visible: fidelity_err_rad below
        # carries the true error, and /ik_feasibility publishes it.
        backstop = 1.0
        if cfg.backstop_max_vx > 0.0 and abs(vx * gain) > cfg.backstop_max_vx:
            backstop = min(backstop, cfg.backstop_max_vx / abs(vx * gain))
        if cfg.backstop_max_wz > 0.0 and abs(wz * gain) > cfg.backstop_max_wz:
            backstop = min(backstop, cfg.backstop_max_wz / abs(wz * gain))
        also_limited_by = "none"
        if backstop < 1.0:
            gain *= backstop
            # Keep whatever was limiting before, rather than discarding it:
            # "backstop" alone cannot tell you whether a transition was also
            # being protected at the time.
            also_limited_by = limited_by
            limited_by = "backstop"

        gain = min(1.0, max(0.0, gain))
        self._last_cmd = cmd
        scaled = (vx * gain, vy * gain, wz * gain)
        return ShapedTwist(
            vx=scaled[0], vy=scaled[1], wz=scaled[2],
            gain=gain,
            feasible=delta <= g.ONE_TICK_BETA,
            limited_by=limited_by,
            also_limited_by=also_limited_by,
            # The geometry error this output actually carries. Inside
            # icr_fidelity_tol_rad whenever the guard set the gain; possibly
            # well outside it when the backstop overrode the guard. Reported
            # either way - a limit that silently misbehaves is worse than one
            # that says what it cost.
            fidelity_err_rad=g.command_distance(cmd, scaled),
            delta_beta_rad=delta,
            hold_remaining_s=self._hold_remaining_s,
            icr=icr,
            straight_bias_rad_s=self._straight_bias(cmd, icr),
        )

    # -- the fidelity guard -------------------------------------------------
    def _min_faithful_gain(self, cmd) -> float:
        """The smallest gain whose own geometry error stays inside tolerance.

        Scaling a twist is only geometry-preserving for a pure point turn. The
        IK's delta = 0.005 rad/s floor on the yaw rate means a scaled-down
        near-straight command reads as a much tighter arc: (0.05, 0, 0) at 20%
        moves the ICR from 9.87 m to 2.0 m. The error is monotone decreasing in
        the gain, so bisection finds the boundary.
        """
        tol = self.config.icr_fidelity_tol_rad
        lo, hi = self.config.min_gain, 1.0
        if self._geometry_error(cmd, lo) <= tol:
            return lo
        for _ in range(24):
            mid = 0.5 * (lo + hi)
            if self._geometry_error(cmd, mid) <= tol:
                hi = mid
            else:
                lo = mid
        return hi

    @staticmethod
    def _geometry_error(cmd, gain: float) -> float:
        scaled = (cmd[0] * gain, cmd[1] * gain, cmd[2] * gain)
        return g.command_distance(cmd, scaled)

    @staticmethod
    def _straight_bias(cmd, icr) -> float:
        """The yaw the IK adds to a command that asked for none.

        Reported, not corrected: correcting it would mean increasing |wz|,
        which the never-larger rule forbids. SP12 yard-tuning input.
        """
        if cmd[2] != 0.0:
            return 0.0
        radius = math.hypot(icr[0], icr[1])
        if radius < 1e-9:
            return 0.0
        return math.hypot(cmd[0], cmd[1]) / radius
```

- [ ] Run `test_shaper.py` and `test_golden_parity.py`. Both green. If `test_hold_table_bounds_every_measured_transition` fails, the fixture is the authority: raise the offending `HOLD_TICKS` bucket to the measured maximum, never lower the assertion.
- [ ] Commit: `git add rover/src/navi_shaper/navi_shaper/shaper.py rover/src/navi_shaper/test/test_shaper.py` then commit as `SP10 Task 3: the gain policy, with hold times measured from the 2.42 model`.

---

## Task 4: The `twist_shaper` node

**Files:**
- Create `rover/src/navi_shaper/navi_shaper/twist_shaper.py`
- Test: create `rover/src/navi_shaper/test/test_twist_shaper.py`

### Steps

- [ ] **Write the test first.** `test/test_twist_shaper.py` — a real ROS graph on domain 92, following `test_mode_supervisor.py`'s shape:

```python
"""The node. A real graph on a throwaway domain - never domain 0, and
/manual_twist is never created here."""
import json
import os

os.environ["ROS_DOMAIN_ID"] = "92"

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from navi_shaper.twist_shaper import TwistShaperNode


class FakeClock:
    """The node's clock, under the test's control.

    Wall time cannot be used here. The shaper's hold windows are seconds long
    and the node measures dt from this clock, so a test that relies on how long
    `spin_once` happens to take is a test that fails on a loaded machine. The
    node takes `clock` for exactly this reason.
    """

    def __init__(self):
        self.t = 0.0

    def advance(self, dt):
        self.t += dt

    def __call__(self):
        return self.t


@pytest.fixture
def graph():
    rclpy.init()
    clock = FakeClock()
    node = TwistShaperNode(clock=clock)
    probe = rclpy.create_node("sp10_probe")
    received = []
    status = []
    probe.create_subscription(Twist, "/chassis_twist", lambda m: received.append(m), 10)
    probe.create_subscription(String, "/ik_feasibility",
                              lambda m: status.append(json.loads(m.data)), 10)
    pub = probe.create_publisher(Twist, "/rover_twist", 10)

    def send(vx, vy, wz, dt=0.05, spins=8):
        clock.advance(dt)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        pub.publish(msg)
        for _ in range(spins):
            rclpy.spin_once(node, timeout_sec=0.02)
            rclpy.spin_once(probe, timeout_sec=0.02)

    def settle(vx, vy, wz):
        """Two messages with a long gap between them.

        The second arrives with dt far above max_dt_s, which the shaper reads
        as "the chassis has long since finished moving" and treats as settled -
        the same path a dropped link takes. Cheaper and more deterministic than
        pumping 72 messages through a real graph to time a hold out.
        """
        send(vx, vy, wz)
        send(vx, vy, wz, dt=10.0)

    yield node, probe, send, settle, received, status
    node.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def test_it_does_not_publish_rover_twist(graph):
    """The single-writer rule. The shaper subscribes; it must never publish."""
    node, _, _, _, _, _ = graph
    names = [n for n, _ in node.get_publisher_names_and_types_by_node(
        node.get_name(), node.get_namespace())]
    assert "/rover_twist" not in names
    assert "/chassis_twist" in names


def test_a_settled_command_is_relayed_unchanged(graph):
    _, _, _, settle, received, _ = graph
    settle(0.05, 0.0, 0.1)
    assert received
    assert received[-1].linear.x == pytest.approx(0.05)
    assert received[-1].angular.z == pytest.approx(0.1)


def test_zeros_are_relayed_immediately_and_exactly(graph):
    _, _, send, settle, received, _ = graph
    settle(0.05, 0.0, 0.0)
    before = len(received)
    send(0.0, 0.0, 0.0)
    assert len(received) > before, "a stop must produce an output message"
    assert (received[-1].linear.x, received[-1].linear.y, received[-1].angular.z) == (0.0, 0.0, 0.0)


def test_one_output_per_input_no_resampling(graph):
    _, _, send, _, received, _ = graph
    received.clear()
    for _ in range(10):
        send(0.05, 0.0, 0.1)
    assert len(received) == 10


def test_a_geometry_jump_is_shaped_not_clipped(graph):
    _, _, send, settle, received, _ = graph
    settle(0.05, 0.0, 0.0)
    received.clear()
    send(0.0, 0.0, 0.1)
    out = received[-1]
    assert out.angular.z != 0.0, "shaped, not clipped - the command still gets through"
    assert abs(out.angular.z) < 0.1, "but scaled down while the wheels catch up"
    assert out.angular.z > 0.0, "and never with a flipped sign"
    assert out.angular.z == pytest.approx(0.1 * 0.106, abs=0.002)


def test_feasibility_status_is_json_with_the_agreed_keys(graph):
    node, probe, send, settle, _, status = graph
    settle(0.05, 0.0, 0.0)
    send(0.0, 0.0, 0.1)
    node._publish_status()
    for _ in range(10):
        rclpy.spin_once(probe, timeout_sec=0.02)
    assert status
    s = status[-1]
    assert set(s) >= {"gain", "feasible", "limited_by", "also_limited_by",
                      "fidelity_err_rad", "delta_beta_rad",
                      "hold_remaining_s", "icr_x", "icr_y", "shaped_count",
                      "straight_bias_rad_s"}
    assert 0.0 <= s["gain"] <= 1.0
    assert isinstance(s["feasible"], bool)
    assert s["limited_by"] == "slew"
    assert s["shaped_count"] >= 1


def test_parameters_are_declared_and_change_behaviour(graph):
    node, _, send, settle, received, _ = graph
    node.set_parameters([rclpy.parameter.Parameter(
        "backstop_max_vx", rclpy.Parameter.Type.DOUBLE, 0.01)])
    received.clear()
    settle(0.05, 0.0, 0.0)
    # 0.01 / 0.05 = a backstop gain of 0.2, and the backstop is a hard ceiling
    # that deliberately overrides the fidelity guard - so this settles at 0.01
    # m/s even though a gain of 0.2 carries about 0.2365 rad of geometry error,
    # well past the 0.10 rad default tolerance. That is the documented
    # precedence, not a bug; the error is reported, not hidden.
    assert abs(received[-1].linear.x) <= 0.01 + 1e-9


def test_an_overriding_backstop_reports_its_true_geometry_error(graph):
    """The backstop wins over the fidelity guard, and says so honestly."""
    node, probe, send, settle, _, status = graph
    node.set_parameters([rclpy.parameter.Parameter(
        "backstop_max_vx", rclpy.Parameter.Type.DOUBLE, 0.01)])
    settle(0.05, 0.0, 0.0)
    node._publish_status()
    for _ in range(10):
        rclpy.spin_once(probe, timeout_sec=0.02)
    s = status[-1]
    assert s["limited_by"] == "backstop"
    # Not clipped to the tolerance, and not silently zero: the payload carries
    # the error the output really has, so an out-of-tolerance backstop is
    # visible on /ik_feasibility rather than only in the rover's tracks.
    assert s["fidelity_err_rad"] > 0.10
```

- [ ] Run it. Red.
- [ ] **Write `navi_shaper/twist_shaper.py`:**

```python
"""The feasibility clamp, as a node.

Subscribes /rover_twist (mode_supervisor is its only publisher - see SP5),
shapes each message with navi_shaper.shaper, and publishes the result on
/chassis_twist, which is what bema_bridge consumes. Diagnostics go out on
/ik_feasibility as JSON in a std_msgs/String, the convention /mode_status,
/drive_status and /localization/status already follow, so the ground station
can read it over rosbridge with no ROS installed.

The shaping happens synchronously in the subscription callback. There is no
timer on the twist path: this node is a transform, not a resampler, and adding
a tick would add up to a tick of latency to the e-stop's zero stream for no
benefit. One message in, one message out, in the same callback.

If this node dies, /chassis_twist goes silent and bema_bridge's own 1 s
deadman stops the rover. That is the intended failure mode and it is why
start_navi.sh starts this before the bridge.
"""
import json
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from navi_shaper.shaper import ShaperConfig, TwistShaper

STATUS_HZ = 2.0


class TwistShaperNode(Node):

    def __init__(self, clock=monotonic, parameter_overrides=None):
        super().__init__("twist_shaper",
                         parameter_overrides=parameter_overrides or [])
        defaults = ShaperConfig()
        self.declare_parameter("min_gain", defaults.min_gain)
        self.declare_parameter("icr_fidelity_tol_rad", defaults.icr_fidelity_tol_rad)
        self.declare_parameter("backstop_max_vx", defaults.backstop_max_vx)
        self.declare_parameter("backstop_max_wz", defaults.backstop_max_wz)
        self.declare_parameter("max_dt_s", defaults.max_dt_s)
        self.declare_parameter("input_topic", "/rover_twist")
        self.declare_parameter("output_topic", "/chassis_twist")

        # NOT self._clock: rclpy.node.Node already owns that name, and
        # create_timer() defaults clock=self._clock - overwriting it makes
        # every timer raise AttributeError on a plain callable. The same trap
        # SP5 documented in mode_supervisor.
        self._now = clock
        self._last_stamp = None
        self._shaper = TwistShaper(self._config())
        self._last_result = None
        self._shaped_count = 0

        out_topic = str(self.get_parameter("output_topic").value)
        self._twist_pub = self.create_publisher(Twist, out_topic, 1)
        self._status_pub = self.create_publisher(String, "/ik_feasibility", 1)
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._on_twist, 1)
        self.create_timer(1.0 / STATUS_HZ, self._publish_status)

        self.get_logger().info(
            f"twist_shaper: {self.get_parameter('input_topic').value} -> {out_topic}; "
            f"backstop {self.get_parameter('backstop_max_vx').value} m/s / "
            f"{self.get_parameter('backstop_max_wz').value} rad/s")

    def _config(self) -> ShaperConfig:
        return ShaperConfig(
            min_gain=float(self.get_parameter("min_gain").value),
            icr_fidelity_tol_rad=float(self.get_parameter("icr_fidelity_tol_rad").value),
            backstop_max_vx=float(self.get_parameter("backstop_max_vx").value),
            backstop_max_wz=float(self.get_parameter("backstop_max_wz").value),
            max_dt_s=float(self.get_parameter("max_dt_s").value),
        )

    def _on_twist(self, msg: Twist):
        # Parameters are read per message so a live `ros2 param set` takes
        # effect at once; the shaper's hold state is carried across, because
        # the chassis does not reset when a parameter does.
        self._shaper.config = self._config()

        now = self._now()
        dt = 0.0 if self._last_stamp is None else max(0.0, now - self._last_stamp)
        self._last_stamp = now

        result = self._shaper.shape(msg.linear.x, msg.linear.y, msg.angular.z, dt)
        self._last_result = result
        if result.gain < 1.0:
            self._shaped_count += 1

        out = Twist()
        out.linear.x = result.vx
        out.linear.y = result.vy
        out.angular.z = result.wz
        # linear.z and angular.x/y are left at zero: the chassis has no such
        # degrees of freedom and mode_supervisor never populates them.
        self._twist_pub.publish(out)

    def _publish_status(self):
        r = self._last_result
        payload = {
            "gain": 1.0 if r is None else round(r.gain, 6),
            "feasible": True if r is None else bool(r.feasible),
            "limited_by": "none" if r is None else r.limited_by,
            # What else was pushing the gain down when the backstop overrode
            # it. "backstop" alone would hide whether the slew policy or the
            # fidelity guard was also active, and ruling 4 justifies
            # limited_by as "the whole content of the diagnostic".
            "also_limited_by": "none" if r is None else r.also_limited_by,
            # The geometry error the shaped output actually carries. The
            # backstop is applied after the fidelity bisection and is allowed
            # to exceed icr_fidelity_tol_rad (see the ordering rule in
            # shaper.py), so this is the only place the true error is visible.
            "fidelity_err_rad": 0.0 if r is None else round(r.fidelity_err_rad, 6),
            "delta_beta_rad": 0.0 if r is None else round(r.delta_beta_rad, 6),
            "hold_remaining_s": 0.0 if r is None else round(r.hold_remaining_s, 3),
            "icr_x": 0.0 if r is None else round(r.icr[0], 4),
            "icr_y": 0.0 if r is None else round(r.icr[1], 4),
            "straight_bias_rad_s": 0.0 if r is None else round(r.straight_bias_rad_s, 6),
            "shaped_count": self._shaped_count,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistShaperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] Build and test: `cd rover && colcon build --packages-select navi_shaper && source install/local_setup.bash && ROS_DOMAIN_ID=92 python3 -m pytest src/navi_shaper/test -q`. All green.
- [ ] Verify by hand that the whole rover workspace still builds: `cd rover && colcon build`. No task may leave the build red.
- [ ] Commit: `git add rover/src/navi_shaper/navi_shaper/twist_shaper.py rover/src/navi_shaper/test/test_twist_shaper.py` then commit as `SP10 Task 4: the twist_shaper node and /ik_feasibility`.

---

## Task 5: Wire it into `start_navi.sh`

Parameter change only — no edit to `bema_bridge.py`, exactly the mechanism SP5 used.

**Files:**
- Modify `rover/start_navi.sh`
- Modify `rover/test/test_start_navi_gate.sh`

### Steps

- [ ] **Extend the gate test first.** `rover/test/test_start_navi_gate.sh` already sources `start_navi.sh` with `NAVI_FUNCTIONS_ONLY=1` and a fake `ros2`. Add cases asserting, by grepping the script text (the launch section is below the `NAVI_FUNCTIONS_ONLY` return and cannot be sourced):
  - `start_navi.sh` contains `twist_topic:=/chassis_twist` and does **not** contain `twist_topic:=/rover_twist`;
  - the `twist_shaper` launch line appears **before** the `bema_bridge` line (`awk` on line numbers);
  - `--no-shaper` is accepted by the argument loop and sets `START_SHAPER=0`;
  - the stale-cleanup section names `navi_shaper/twist_shaper`.
- [ ] **Edit `rover/start_navi.sh`:**
  - Add to the usage header:
    ```
    #   ./start_navi.sh --no-shaper   no twist_shaper (bema_bridge then has no
    #                                 publisher on /chassis_twist and the rover
    #                                 cannot be driven - for bench work only)
    ```
  - Add `START_SHAPER=1` beside the other flags and `--no-shaper) START_SHAPER=0; shift ;;` to the argument loop.
  - Extend the stale cleanup, in the existing style:
    ```bash
    kill_stale "navi_shaper nodes" "navi_shaper/twist_shaper"
    ```
    and widen the existing `ros2 run` wrapper pattern to `"ros2 run navi_(teleop|supervisor|shaper)"`.
  - Between the supervisor block and the drive-bridge block:
    ```bash
    if [ "$START_SHAPER" -eq 1 ]; then
        # After the supervisor and before the bridge: each consumer's source
        # must have a publisher by the time it subscribes. This node is the
        # feasibility clamp - it turns /rover_twist into a twist the real 2.42
        # IK can execute and republishes it on /chassis_twist.
        echo "starting twist_shaper (/rover_twist -> /chassis_twist)"
        ros2 run navi_shaper twist_shaper &
        BACKGROUND_PIDS+=("$!")
    fi
    ```
  - Change the drive-bridge block to consume `/chassis_twist`, and warn loudly when the shaper is off — never silently fall back to `/rover_twist`, because an unclamped path to the wheels is exactly the mistake this whole sub-project exists to prevent:
    ```bash
    if [ "$START_DRIVE_BRIDGE" -eq 1 ]; then
        if [ "$START_SHAPER" -eq 0 ]; then
            echo "warning: --no-shaper with the drive bridge running." >&2
            echo "         Nothing publishes /chassis_twist, so the bridge will" >&2
            echo "         see no twist and its 1 s deadman will hold the rover" >&2
            echo "         stopped. This is deliberate: the bridge is NOT" >&2
            echo "         re-pointed at /rover_twist, because an unclamped path" >&2
            echo "         to the wheels is the failure SP10 exists to prevent." >&2
        fi
        # The feasibility-clamped stream, not /rover_twist. Passed explicitly
        # so the wiring reads at the launch site rather than only in the node.
        echo "starting bema_bridge on /chassis_twist (idle until something drives)"
        ros2 run navi_teleop bema_bridge --ros-args -p twist_topic:=/chassis_twist &
        BACKGROUND_PIDS+=("$!")
    fi
    ```
  - Update the numbered node list in the usage header: insert `twist_shaper` as item 6 and renumber, describing it as "the feasibility clamp: /rover_twist -> /chassis_twist, plus /ik_feasibility".
- [ ] Run `bash rover/test/test_start_navi_gate.sh`. Green. Then `bash -n rover/start_navi.sh` for a syntax check.
- [ ] Commit: `git add rover/start_navi.sh rover/test/test_start_navi_gate.sh` then commit as `SP10 Task 5: start twist_shaper and feed bema_bridge from /chassis_twist`.

---

## Task 6: Path-following verification in the sim

The end-to-end check: a scripted twist sequence goes through the real shaper into the real 2.42 IK, and the executed trajectory is compared with the commanded one. Laptop-side, headless, domain 91. The chassis is not involved and `/manual_twist` is never created.

**Files:**
- Create `sim/test/sp10_path_following.sh` (`sim/test/` is a new directory — `sim/` currently contains only `src/`)

### Steps

- [ ] **Write `sim/test/sp10_path_following.sh`.** It runs three nodes on `ROS_DOMAIN_ID=91`: `twist_shaper` (from the rover workspace) remapped to a scratch input/output pair, `sim_ik_node` (from the sim workspace) remapped from `/manual_twist` to the shaper's output, and a scripted publisher. It then reads `/sim_odom` and asserts. Both workspaces must be built first; the script checks for `sim/install/local_setup.bash` and `rover/install/local_setup.bash` and exits with the `colcon build` command to run if either is missing, rather than failing later with an opaque "package not found".

  Structure, with the parts that matter spelled out:

  ```bash
  #!/usr/bin/env bash
  # SP10 path-following verification. Laptop-side, headless, no Gazebo GUI and
  # no chassis: sim_ik_node integrates the pose from the real 2.42 model's
  # eta_dot_constrained, which is precisely the trajectory the wheels would
  # produce. Domain 91 throughout; /manual_twist is remapped away and never
  # created, so nothing here can reach the rover.
  set -eo pipefail
  export ROS_DOMAIN_ID=91
  REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  source /opt/ros/humble/setup.bash
  source "$REPO_DIR/sim/install/local_setup.bash"
  source "$REPO_DIR/rover/install/local_setup.bash"

  # A PATH pattern for the shaper, not `-x`. twist_shaper is a setuptools
  # console script installed to lib/navi_shaper/twist_shaper with a
  # #!/usr/bin/python3 shebang, so the kernel's `comm` for the process is
  # `python3` - an exact-name kill on twist_shaper matches nothing, ever, and
  # would leave run 1's shaper alive to contaminate the unshaped baseline.
  # The path form is the idiom start_navi.sh's own kill_stale already uses
  # (pgrep -f "navi_supervisor/mode_supervisor"): it matches the console
  # script's full command line and cannot match the shell running this
  # script. sim_ik_node is a real compiled executable, so -x is right there.
  cleanup() {
    pkill -f 'navi_shaper/twist_shaper' || true
    pkill -x sim_ik_node || true
  }
  trap cleanup EXIT

  ros2 run navi_shaper twist_shaper --ros-args \
    -p input_topic:=/sp10_cmd -p output_topic:=/sp10_chassis \
    -p backstop_max_vx:=0.05 -p backstop_max_wz:=0.1 &
  ros2 run navi_sim_ik sim_ik_node --ros-args -r /manual_twist:=/sp10_chassis &
  sleep 4
  ```

  Then a Python driver (inline via `python3 - <<'PY'`) that publishes the scripted sequence to `/sp10_cmd` at 20 Hz and records `/sim_odom`:

  | phase | duration | command | what it exercises |
  |---|---|---|---|
  | 1 | 6 s | `(0.05, 0, 0)` | straight, settled — the relay case |
  | 2 | 6 s | `(0.05, 0, 0.1)` | RPP-regime arc, reached by a **ramp** over 1 s | 
  | 3 | 6 s | `(0.0, 0, 0.1)` | **step** to a point turn — the infeasible jump |
  | 4 | 6 s | `(0.05, 0, 0)` | **step** back to straight — the fidelity-limited case |

- [ ] **Assertions**, each with its reason:
  - **The jump is shaped, not clipped.** During the first second of phase 3, `/sp10_chassis.angular.z` is strictly between 0 and 0.1 — nonzero (the command still gets through) and reduced (the chassis is being protected). Assert `0.0 < z < 0.05` at the first message after the step, and `z == 0.1` by the end of phase 3.
  - **The ramp is not shaped.** Across the whole of phase 2, `/sp10_chassis` equals `/sp10_cmd` to 1e-9. This is spec §5's claim, verified end to end: a curvature-continuous command is passed through untouched.
  - **Zeros are exact.** Publish `(0,0,0)` between phases 3 and 4 for 0.5 s and assert every output message is exactly zero and that the output count matches the input count (no dropped or delayed stop).
  - **Path tracking, measured cross-track and never along-track.** Integrate the *commanded* twist (dead reckoning at 20 Hz) into a reference **path**, then measure the perpendicular distance from each `/sim_odom` pose to that polyline. Cross-track, not point-to-point at matched times: the shaper deliberately makes the rover lag along the path — that lag *is* the mechanism — so an along-track comparison would report the shaper working as if it were failing. Assert peak cross-track error **below 0.05 m** over phases 1–2 and **below 0.25 m** over phases 3–4, with the final pose within 0.25 m of the path. The looser bound on 3–4 is honest: the shaper reduces the transition excursion, it does not eliminate it (measured, at the manual cap, an unshaped straight → point-turn sweep covers 0.056 m of wrong-geometry ground; the shaper cuts it to about 0.014 m).
  - **Phase 1 starts shaped, because both the shaper and `sim_ik_node` have just booted.** This is a *boot* effect, not a stop effect. A freshly `initialize()`d 2.42 model has `UnitDelay_DSTATE = [0,0,0]`, so before it has ever seen a non-zero command its retained geometry really is the point turn — and `reset()` gives the shaper the matching belief, `_last_cmd = (0,0,0)`. The first straight command therefore opens a full 3.6 s hold. It is the only time that happens: per ruling 5(a) a *stop* retains the previous geometry on both sides, so a resume mid-run is free. Phase 1 is 6 s so the boot hold clears with margin; assert the gain reaches 1.0 before phase 1 ends.
  - **The zero burst does not erase the geometry.** After the `(0,0,0)` burst between phases 3 and 4, phase 4's step back to straight must still be shaped against the **point-turn** geometry phase 3 left behind — `limited_by: "fidelity"` at gain ≈ 0.35, not an unshaped gain of 1.0. This is the end-to-end form of ruling 5(a): the model's `'<S1>/Retain Translation'` block held the wheels in the point-turn pose right through the stop, and the shaper's belief held with it.
  - **Shaping actually helped.** Run the same script twice — once normally, once with `-p min_gain:=1.0` (which pins the gain at 1.0 and disables shaping) — and assert the peak cross-track error over phases 3–4 is **lower with shaping on**. This is the acceptance criterion of the sub-project: a comparison against the unshaped baseline, not an absolute number that could be met by a node that does nothing. The baseline arm is only valid if run 1's shaper is genuinely dead first — see the teardown assertion in the next step.
  - **The status topic reflects reality.** `ros2 topic echo /ik_feasibility --once` at the start of phase 3 shows `"limited_by": "slew"` with `gain` near 0.11 (a point turn's geometry is scale-invariant, so the fidelity guard does not lift it); at the start of phase 4 it shows `"limited_by": "fidelity"` with `gain` near 0.35 (the guard refusing to curve a straight command any further). Both settle to `gain: 1.0`, `"limited_by": "none"` before their phase ends.
  - **`/manual_twist` does not exist.** The remap is what makes that true, so it is checked rather than assumed. Write it as an explicit `if`, **not** as a bare `grep -qx` whose failure is the assertion — under the `set -eo pipefail` at the top of the script a failing command *is* an abort, with no message and a bare non-zero exit, which is the exact trap `start_navi.sh:107-110` already documents:
    ```bash
    if ros2 topic list | grep -qx /manual_twist; then
        echo "FAIL: /manual_twist exists - the remap did not take" >&2
        exit 1
    fi
    ```
- [ ] Stop the nodes with `pkill -f 'navi_shaper/twist_shaper'` and `pkill -x sim_ik_node`, then **assert the teardown actually worked before the second run starts**: `! pgrep -f 'navi_shaper/twist_shaper' >/dev/null`. A path pattern for the console script and `-x` for the compiled executable; never a `pkill -f` pattern that would match the shell running the script. The assertion is not decoration — without it a surviving shaper from run 1 publishes to `/sp10_chassis` alongside run 2's unshaped one, `sim_ik_node` integrates the interleaved mixture, and the A/B comparison is measured against garbage that is *likely to appear to pass*, because the surviving shaped stream drags the baseline's error down.
- [ ] Run the script. It must exit 0 and print a one-line summary per assertion. Record the measured peak cross-track errors (shaped and unshaped) in the commit message **and write them into `sim/test/sp10_path_following.sh` as asserted upper bounds**, with generous margin — two live 20 Hz ROS runs are not bit-deterministic, so bound them at roughly twice the measured value rather than at it. These are explicitly SP12's yard-tuning baseline, and a baseline that lives only in a commit message is a baseline nobody checks: as asserted bounds, a future regression fails the script instead of requiring someone to read git history.
- [ ] Update `sim/src/navi_sim_ik/test/feasibility_harness_242.cpp`'s header comment with a pointer to this script, so the harness and its consumer are discoverable from each other.
- [ ] Commit: `git add sim/test/sp10_path_following.sh` then commit as `SP10 Task 6: sim path-following verification through the shaper`.

---

## Done when

- `python3 -m pytest rover/src/navi_shaper/test -q` is green, including the golden-parity test that pins the pure map to the real 2.42 model at 1e-12 and asserts the hold table bounds all 342 measured transitions **and all 324 retained (drive → stop → command) transitions**.
- The stop semantics are the model's, asserted both ways: a stop retains the chassis geometry (`icr_drift_during_stop == 0.0` on every retained row), so resuming the same command is a pure relay at gain 1.0, while **settled straight → stop → point turn is still shaped at gain ≈ 0.106 with the full 60-tick hold**. The zero path relays the stop untouched and immediately without clearing `_last_cmd`.
- `cd rover && colcon build` and `cd sim && colcon build` are both green.
- `bash rover/test/test_start_navi_gate.sh` is green, and `start_navi.sh` starts `twist_shaper` between the supervisor and the bridge with the bridge on `/chassis_twist`.
- `bash sim/test/sp10_path_following.sh` exits 0, with a measurably lower peak cross-track error through the geometry transitions than the same run with shaping disabled — the baseline arm run against a *verified-dead* shaper (`! pgrep -f 'navi_shaper/twist_shaper'` between the two runs), and both peak errors asserted as bounds in the script rather than recorded only in the commit message.
- Nothing publishes `/rover_twist` except `mode_supervisor`; nothing anywhere published `/manual_twist` during any test.

## Handed forward

- **SP12 (yard tuning)** gets three things measured here: the hold-time table to re-measure against the real steering slew (`kBetaDotMax = 1.5` is `IkController.h`'s number, not a measured one — risk 3 in §11); the straight-line ICR bias reported on `/ik_feasibility`, which no shaper can correct and which grows as speed falls; and the sim path-following baseline from Task 6.
- **SP11 (ground station NAV row)** can subscribe `/ik_feasibility` over rosbridge for free — it is JSON in a `std_msgs/String` like every other status topic. Not in this plan's scope.
- **SP9 (Nav2)** owns the velocity smoother that sets the §10 cap. The shaper's `backstop_max_vx` / `backstop_max_wz` shadow it and must be raised in step with it, following §10's ladder and only on explicit instruction.
