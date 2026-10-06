# explore-ros2 — Radiation-Aware Frontier Exploration (RaMonES fork)

This repository is a **fork of the `m-explore-ros2` frontier-exploration package**
(the ROS 2 port of Jiri Horner's `m-explore` / `explore_lite` WFD explorer, ported by
Carlos Alvarez and Juan Galvis), extended for the **RaMonES** radiation-aware
autonomous exploration system.

In RaMonES the robot builds a Gaussian-Process (GP) radiation field online from a
GM-counter stream; this package is the **closed-loop explorer** (dissertation
Chapter 5). It consumes the predicted radiation field (GP mean and GP standard
deviation, published as images) and folds it into frontier selection, and it is run
in two arms:

* **RaMonES arm** — radiation-informed frontier costing enabled.
* **Baseline arm** — radiation weights zeroed at launch (`radiation_agnostic:=true`),
  giving a radiation-agnostic greedy explorer for comparison.

It contains two ROS 2 packages:

| Package (`package.xml` name) | Directory | Status |
|---|---|---|
| `explore_lite` | `explore/` | **Heavily modified** by RaMonES — the subject of this document |
| `multirobot_map_merge` | `map_merge/` | Upstream m-explore port, **unchanged**; not used by RaMonES |

> The `multirobot_map_merge` package is a direct upstream port with no RaMonES
> changes and is not part of the single-robot RaMonES pipeline. It is left in the
> tree for completeness. Everything below concerns `explore_lite`.

---

## 1. What is a fork modification vs. upstream

The upstream `explore_lite` is a greedy frontier explorer: a Dijkstra/BFS frontier
search over the Nav2 global costmap, a cost function of distance minus a size gain,
a goal blacklist, and a `NavigateToPose` action client driven on a timer.

The RaMonES fork keeps that skeleton (`costmap_client.cpp`, the `FrontierSearch`
BFS/flood-fill, the timer-driven `makePlan()` loop, the blacklist, the action
client) and adds the following. **All of the radiation, battery, dose-budget,
exploration-mode, and recovery machinery below is original to this fork.**

Original source files of interest:

* `explore/src/frontier_search.cpp` + `include/explore/frontier_search.h` — frontier
  detection and the **cost function** (all radiation/battery/dose/orientation terms).
* `explore/src/explore.cpp` + `include/explore/explore.h` — the `explore_node`:
  topic/heatmap plumbing, exploration-mode state machine, goal management, recovery,
  services.
* `explore/src/costmap_client.cpp` — costmap/TF client (upstream, unchanged in substance).

### Executables / nodes

A single executable is built: **`explore`** (CMake target `explore`, source
`costmap_client.cpp` + `explore.cpp` + `frontier_search.cpp`), run as node
**`explore_node`** (see `explore.launch.py`). There is no separate "radiation node";
the radiation logic lives inside `explore_node`.

---

## 2. The frontier cost function (`FrontierSearch::frontierCost`)

Frontiers are ranked by a single additive cost (lower is better); the explorer picks
the lowest-cost feasible frontier. The terms, in source order:

```
cost = original_cost          # potential_scale * path_distance - capped size gain
     + info_gain_cost         # -gain_scale * visible-unknown cells (raycast), capped
     + rad_cost               # radiation avoidance penalty  (RaMonES)
     + seek_bonus             # radiation attraction bonus    (RaMonES, <= 0)
     + dose_budget_cost       # transit-dose efficiency cost   (RaMonES)
     + orientation_penalty    # two-tier forward/rear heading preference
     + battery_penalty        # energy-to-home penalty         (RaMonES)
     + hysteresis_bonus       # small discount near current goal
```

Two **hard gates** return `+inf` (frontier removed from consideration) before the sum:

1. **Lethal-point ceiling** — if the GP mean at the frontier centroid *or* its closest
   accessible "middle" point is `>= lethal_point_ceiling_mrem_hr`, the frontier is
   rejected. The caller passes the *effective acute threshold* here (see §4), not a
   fixed constant.
2. **Transit-dose affordability** — the expected round-trip dose to the frontier
   (centroid dose rate × round-trip time at an assumed 0.5 m/s) must not exceed
   `frontier_spend_fraction × remaining_dose_budget`.

### Radiation terms (RaMonES)

The GP mean image (`heatmap`) and GP std image (`heatmap_std`) are sampled at the
frontier centroid. Note the **image row flip**: costmap `y` increases upward, image
row increases downward, so `row = (heatmap.rows - 1) - my`.

* **`radiation_scale` (avoidance penalty).** Below a physical warning threshold of
  `0.25` mrem/hr the penalty is linear (`radiation_scale * radiation`); above it the
  penalty is **quadratic** (`radiation_scale * (radiation/0.25)^2`) so avoidance grows
  faster than any gain term can offset as the field climbs.
* **`radiation_gradient_scale` (gradient term).** Central-difference GP-mean gradient
  dotted with the unit robot→frontier direction, so the cost rewards/penalizes moving
  with or against the dose gradient.
* **`radiation_seek_scale` (attraction / `seek_bonus`).** A negative (cost-reducing)
  information-gain term:
  `seek_bonus = -radiation_seek_scale * elevation * GP_std`, where
  `elevation = max(0, GP_mean - 0.25)`. It rewards frontiers that are **both** elevated
  above background **and** still uncertain in the GP, drawing the robot to linger near
  an *unconfirmed* source long enough to localize it. Properties:
  - **Capped** at 25 % of the frontier's own distance cost (mirrors the size-gain cap),
    so a far "merely curious" frontier can never beat a close one on radiological
    interest alone.
  - Fades to ~0 once a region is back at background (elevation → 0) or the GP has
    converged (std → 0); it cannot trap the robot orbiting a characterized source.
  - **Never touches the two hard gates** — safety always wins.
  - **Trap-radius denial:** if `trap_avoid_radius_m > 0`, the bonus is set to 0 for any
    frontier whose centroid lies within `trap_avoid_radius_m` of a recent stuck-escape
    location (see §3). This stops a stall from reinforcing attraction toward a goal
    behind a physical obstacle the robot has repeatedly failed to clear.
  - **Adaptive / displacement-based variant:** when `radiation_seek_adaptive=true`, the
    scale actually applied each tick is `radiation_seek_scale × deficit`, where
    `deficit = clamp(1 - net_displacement / stall_min_displacement_m, 0, 1)` over the
    trailing `stall_window_s` seconds (see `Explore::effectiveRadiationSeekScale`). It is
    near-zero during healthy exploration and ramps to full scale only as the robot
    genuinely stalls. This deliberately uses an independent position history, **not**
    the goal-progress timer (which unrelated bookkeeping resets too often to be a
    reliable stall proxy).
* **`dose_budget_scale` (efficiency cost).** When a finite dose budget remains,
  `dose_budget_scale * (expected_transit_dose / remaining_dose_budget)` adds a soft
  preference for cheaper-per-dose frontiers on top of the hard affordability gate.

### Other cost terms (mostly fork-added)

* **Dijkstra path distance.** Frontier distance is the 8-connected Dijkstra cost over
  the costmap (diagonals weighted √2), not Euclidean, so cluttered/aisled maps rank
  correctly. A per-cell *initial step direction* is propagated so the orientation term
  can use the true first move the robot must make.
* **Two-tier orientation penalty.** Any frontier in the rear hemisphere (>90° from
  heading) gets a `1e6` offset so every forward frontier outranks it; forward frontiers
  get a light smooth `orientation_scale`-weighted penalty.
* **Obstacle penalty.** `obstacle_penalty_scale × costmap_cost` on frontiers sitting in
  the inflation buffer.
* **Info-gain raycast.** 36-ray visibility raycast over the explored grid; capped so it
  can offset at most 50 % of the frontier's own distance cost.
* **Battery feasibility + penalty.** If `energy_per_meter > 0` and the battery reading is
  valid, a frontier whose round-trip-to-home energy exceeds `battery_pct − margin` is
  gated to `+inf`; otherwise a `battery_penalty_scale`-weighted penalty applies.
* **Frontier clustering** ("mean frontier candidate") merges frontiers whose centroids
  are within `frontier_clustering_distance`.

---

## 3. The `explore_node` (`explore.cpp`)

### Exploration-mode switching: SPATIAL vs RADIATION

`explore_node` runs a two-state machine (`ExplorationMode::{SPATIAL, RADIATION}`),
starting in SPATIAL. Transitions are driven by the raw dose-rate stream
(`radiationCallback`), which maintains a **Poisson-aware dynamic background threshold**
(`bg_mean + 2.5 × max(bg_std, sqrt(bg_mean))`) from a rolling 50-sample background
buffer (mirroring the heatmap node's logic):

* **SPATIAL → RADIATION** after `radiation_mode_entry_hysteresis` consecutive readings
  above the dynamic threshold (only while exploration is active).
* **RADIATION → SPATIAL** when the rolling mean over `radiation_mode_hysteresis`
  readings drops back below threshold, OR when a `stable_source_captured` message
  arrives (heatmap node committed an averaged source measurement), OR when the boundary
  survey is exhausted.

In **RADIATION mode** `makePlan()` calls `generateRadiationFrontiers()`, which replaces
spatial frontiers with a small set of directed candidates (and re-sorts any spatial
fallbacks to be radiation-aware). Two phases:

* **Phase 1 — gradient ascent.** Smooth the GP mean, take the Sobel gradient at the
  robot, and project ≤3 candidates (straight up-gradient and ±30°) toward the source
  peak at 3 m and 6 m.
* **Phase 2 — boundary survey.** Once the gradient is negligible (peak reached), walk
  radially outward in `radiation_survey_dirs` directions to find where each ray crosses
  from elevated back to background; visit each edge in turn. The survey can be bounded
  either by ray count (`radiation_survey_dirs`) or directly by wall-clock
  (`radiation_survey_time_cap_s`).

The current mode is published, latched, on **`~/mode`** as `std_msgs/String`
(`"spatial_exploration"` / `"radiation_mode"`).

### Goal management, blacklist TTL, trap radius, recovery

* **Blacklist with TTL.** Frontiers that abort, time out, read as blocked, or are
  already-explored-on-arrival are blacklisted (matched within 5 costmap cells). With
  `blacklist_ttl_s > 0`, entries expire after the TTL so an accumulating blacklist can't
  strand the last residual frontiers and complete the mission early at partial coverage
  (the diagnosed `vp_blacklisted` premature-completion path). `blacklist_ttl_s = 0`
  (default) keeps entries permanent.
* **Stuck-escape grace + trap locations.** `explore_node` subscribes to
  `/stuck_escape/event` (published by the external `stuck_escape_node` after its
  backup+spin recovery). A goal failing within `stuck_grace_window_s` of such an event
  gets one extra uninterrupted attempt before blacklisting (bounded globally to 3 grace
  passes). The robot's pose at each stuck event is remembered for
  `radiation_seek_trap_window_s` as a *trap location*, feeding the seek-bonus
  trap-radius denial above.
* **Motion-based progress credit.** A goal's stall timer is also reset whenever the
  robot physically advances `progress_min_motion_m`, so a legitimate detour/replan isn't
  mistaken for a stall.
* **Nav2-recovery retreat ("recalibration").** When Nav2's recovery count increases,
  `makePlan()` cancels the frontier goal and sends a short retreat-to-safe-space goal
  instead of fighting Nav2's own recovery.
* **All-gated completion hold.** If every remaining frontier is infeasible (dose/battery
  gated) rather than absent, completion is held up to
  `all_gated_completion_timeout_s` to let the GP estimate refine, instead of reporting a
  0 m "success".
* **Cumulative-dose / battery return.** Exceeding `max_radiation_dose` (integrated dose)
  or crossing the battery thresholds forces a return to the initial pose (with retry/
  cancel budgets for stuck-escape preemptions).

> **Dead code to note:** the verification phase (`startVerification`,
> `max_verification_attempts`) is **disabled** — `makePlan()` calls
> `handleExplorationCompletion(true)` directly once frontiers are exhausted (it was
> looping indefinitely). The `max_verification_attempts` parameter is therefore inert on
> the live path. `critical_dose_mrem_hr` is likewise retained for semantics but **no
> longer hard-gates** frontier targeting.

### Cross-layer dose-budget contract (§4)

`explore_node` participates in a three-layer dose-budget handoff with the external
`local_recovery_node` and `rad_safety_monitor_node`. Its acute-dose yield threshold and
its per-frontier transit-dose spend fraction are both scaled by a shared multiplier
`min(ceiling, 1 + k × remaining_fraction)`:

* `k = dose_budget_relaxation_k` is the "how freely may budget be spent" knob.
* The remaining fraction prefers the authority value on
  **`/dose_budget/remaining_fraction`** (from `rad_evaluator_node`) when fresh, else this
  node's own dose integration.
* The multiplier actually in use is republished on **`~/dose_budget_multiplier`** for the
  evaluator's per-trial coherence check.

The three layers must relax in lockstep; otherwise a dose band opens in which
`explore_node` has stopped and yielded but no recovery node ever restarts it. This is
why `k` must match the value passed to the recovery nodes.

---

## 4. Topics, services, parameters

### Subscribed topics

| Topic (default) | Type | Role |
|---|---|---|
| `rad` (`radiation_topic`) | `std_msgs/Float64MultiArray` | Instantaneous dose rate; `data[1]` is mR/hr. Drives dose integration + mode switching |
| `/rad_heatmap/raw_mean_map` (`heatmap_topic`) | `sensor_msgs/Image` (32FC1) | **GP mean** radiation field used in frontier costing |
| `/rad_heatmap/gp_std_dev` (`heatmap_std_topic`) | `sensor_msgs/Image` (32FC1) | **GP std-dev** field used by the seek bonus |
| `/battery_state` (`battery_topic`) | `sensor_msgs/BatteryState` | Battery feasibility/penalty (`percentage` on 0–1 scale) |
| `/rad_heatmap/stable_source_captured` | `geometry_msgs/PointStamped` | Heatmap node signalled a source is characterized → exit RADIATION mode |
| `/stuck_escape/event` | `std_msgs/Float64` | External stuck-escape recovery completed → grace + trap location |
| `/dose_budget/remaining_fraction` | `std_msgs/Float64` | Dose-budget authority fraction (cross-layer contract) |
| `explore/resume` | `std_msgs/Bool` | Resume (`true`) / stop (`false`) |

Costmap is consumed via `Costmap2DClient` from `costmap_topic`
(`/global_costmap/costmap`) + `costmap_updates_topic`.

### Published topics

| Topic | Type | Notes |
|---|---|---|
| `~/mode` | `std_msgs/String` | Latched; `spatial_exploration` / `radiation_mode` |
| `~/dose_budget_multiplier` | `std_msgs/Float64` | Current budget multiplier (1 Hz) |
| `explore/is_finished` | `std_msgs/Bool` | Latched, reliable; mission-complete flag |
| `explore/frontiers` | `visualization_msgs/MarkerArray` | Only if `visualize`; spatial frontiers blue→green by info-gain, radiation frontiers purple, blacklisted red |
| `explore/explored_area` | `nav_msgs/OccupancyGrid` | Only if `visualize`; raycast explored grid |

Action client: **`navigate_to_pose`** (`nav2_msgs/action/NavigateToPose`).

### Services

| Service | Type | Action |
|---|---|---|
| `~/start` | `std_srvs/Trigger` | Start/resume exploration (the RaMonES orchestrator calls this after bring-up) |
| `~/stop` | `std_srvs/Trigger` | Pause + cancel current goal (not "finished") |
| `~/return_to_initial_pose` | `std_srvs/Trigger` | Navigate back to the startup pose |
| `~/set_radiation_seeking` | `std_srvs/SetBool` | Force RADIATION (`true`) / SPATIAL (`false`) mode |
| `~/toggle_mode` | `std_srvs/Trigger` | Toggle mode manually (skips entry hysteresis) |
| `~/reset` | `std_srvs/Trigger` | Clear blacklist, zero dose, re-capture initial pose, back to SPATIAL |

### Parameters (declared in `explore.cpp`; defaults as declared)

Values in `explore/config/params.yaml` **override** these node defaults (differences
flagged). The YAML is the configuration actually shipped to the robot.

**Frontier cost weights**

| Param | Node default | params.yaml | Description |
|---|---|---|---|
| `potential_scale` | 1.0 | 10.0 | Distance weight |
| `gain_scale` | 1.0 | 5.0 | Frontier-size / info-gain weight |
| `orientation_scale` | 10.0 | 6.0 | Forward-heading preference |
| `obstacle_penalty_scale` | 1.0 | 2.0 | Inflation-buffer penalty |
| `frontier_clustering_distance` | 3.0 | (3.0) | Merge radius (m) |
| `min_frontier_size` | 0.5 | 0.25 | Min frontier length (m) |
| `min_frontier_clearance` | 0.1 | 1.0 | Clearance required for a frontier cell |

**Planner / goal management**

| Param | Node default | params.yaml | Description |
|---|---|---|---|
| `planner_frequency` | 1.0 | 1.0 | makePlan Hz |
| `progress_timeout` | 30.0 | 20.0 | Seconds w/o progress before abandoning a goal |
| `progress_min_motion_m` | 0.5 | — | Motion that re-credits progress |
| `min_goal_distance_for_replanning` | 0.75 | 0.75 | Suppress replanning within this radius |
| `no_frontier_retry` | 3 | 3 | No-frontier ticks before completing |
| `blacklist_ttl_s` | 0.0 | — | Blacklist entry TTL (0 = permanent) |
| `stuck_grace_window_s` | 20.0 | — | Grace window after a stuck-escape event |
| `return_to_init` | false | true | Return home on completion |
| `auto_start` | true | false | Start exploring on startup (RaMonES waits for `~/start`) |
| `visualize` | false | true | Publish markers + explored grid |

**Radiation / dose**

| Param | Node default | params.yaml | Description |
|---|---|---|---|
| `radiation_scale` | 1.0 | 1.0 | Avoidance penalty weight (zeroed in Baseline arm) |
| `radiation_gradient_scale` | 1.0 | 1.0 | Gradient-term weight (zeroed in Baseline arm) |
| `radiation_seek_scale` | 0.0 | — | Attraction bonus weight (0 = avoidance-only) |
| `radiation_seek_adaptive` | false | — | Ramp seek scale by stall deficit |
| `stall_window_s` / `stall_min_displacement_m` | 30.0 / 1.0 | — | Stall-deficit window/threshold |
| `radiation_seek_trap_avoid_radius_m` | 0.0 | — | Deny seek bonus near stuck-escape locations |
| `radiation_seek_trap_window_s` | 300.0 | — | Trap-location memory |
| `radiation_survey_dirs` | 8 | — | Boundary-survey ray count |
| `radiation_survey_time_cap_s` | 0.0 | — | Boundary-survey wall-clock cap (0 = off) |
| `max_radiation_dose` | 100.0 | 100.0 | Cumulative dose budget (mR) → return home |
| `critical_dose_mrem_hr` | 2.0 | 2.0 | **Retained but no longer gates targeting** |
| `acute_dose_rate_mrem_hr` | 10.0 | 10.0 | Acute rate → stop + yield to recovery (1e6 in Baseline) |
| `dose_budget_scale` | 10.0 | 10.0 | Transit-dose efficiency weight (zeroed in Baseline arm) |
| `safe_dose_threshold_mrem_hr` | 0.5 | 1.0 | "Elevated" threshold for survey crossings |
| `frontier_dose_spend_fraction` | 0.05 | 0.05 | Base transit-dose spend cap per frontier |
| `all_gated_completion_timeout_s` | 180.0 | 180.0 | Hold completion while all frontiers gated |
| `dose_budget_relaxation_k` | 0.0 | — | Shared relaxation coefficient `k` (must match recovery nodes) |
| `dose_budget_relaxation_ceiling_mult` | 2.0 | — | Relaxation ceiling |
| `radiation_mode_entry_hysteresis` | 5 | — | Consecutive above-threshold readings to enter RADIATION |
| `radiation_mode_hysteresis` | 5 | — | Rolling-window size for RADIATION exit |

**Battery**

| Param | Node default | params.yaml | Description |
|---|---|---|---|
| `battery_topic` | `/battery_state` | `/battery_state` | |
| `energy_per_meter` | 0.1 | 0.000185 | Battery fraction/m (YAML value derived from Husky A200 specs; `0` disables battery modelling) |
| `battery_safety_margin` | 0.1 | 0.1 | Return-trip reserve |
| `battery_penalty_scale` | 0.0 | 1.0 | Battery-penalty weight |
| `min_battery_percentage` | 0.1 | — | Low-battery return trigger |
| `max_battery_used` | 0.9 | 0.8 | Max fraction of charge used |

---

## 5. Launch & arms

`explore/launch/explore.launch.py` loads `config/params.yaml` and exposes launch
arguments that are injected **at launch time** (important: `radiation_scale`,
`radiation_gradient_scale`, and `dose_budget_scale` are read once at construction with
**no** `on_set_parameters` callback — a runtime `ros2 param set` has no behavioural
effect):

| Launch arg | Default | Effect |
|---|---|---|
| `use_sim_time` | true | Sim clock |
| `namespace` | `''` | Node namespace |
| `radiation_agnostic` | false | **Baseline arm:** zero `radiation_scale`, `radiation_gradient_scale`, `dose_budget_scale`, and set `acute_dose_rate_mrem_hr=1e6` so frontier selection ignores radiation entirely |
| `dose_relaxation_k` | 0.0 | Sets `dose_budget_relaxation_k` (must match the recovery nodes) |
| `frontier_spend_fraction` | 0.05 | Sets `frontier_dose_spend_fraction` |
| `radiation_seek_scale` | 0.0 | Seek-bonus weight |
| `radiation_seek_adaptive` | false | Adaptive seek ramp |
| `radiation_survey_dirs` | 8 | Boundary-survey rays |
| `radiation_survey_time_cap_s` | 0.0 | Boundary-survey time cap |
| `radiation_seek_trap_avoid_radius_m` | 0.0 | Trap-radius denial |
| `blacklist_ttl_s` | 0.0 | Blacklist TTL |

---

## 6. Dependencies, build, run

Dependencies (`explore/package.xml`): `rclcpp`, `std_msgs`, `sensor_msgs`, `nav_msgs`,
`geometry_msgs`, `visualization_msgs`, `map_msgs`, `nav2_msgs`, `nav2_costmap_2d`,
`tf2`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`, and OpenCV (C++17/14; built with
`ament_cmake`). OpenCV + `cv_bridge` are used to decode the GP heatmap images and to do
the gradient/Sobel work in RADIATION mode.

Build (standard colcon package in the RaMonES `ros_ws`):

```bash
cd <ros_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select explore_lite
source install/setup.bash
```

Run standalone (expects Nav2, a global costmap, the `rad`/heatmap topics, and TF to be
up — in RaMonES these are started by the orchestration launch, and `~/start` is called
after bring-up because `auto_start: false`):

```bash
ros2 launch explore_lite explore.launch.py                      # RaMonES arm
ros2 launch explore_lite explore.launch.py radiation_agnostic:=true   # Baseline arm
ros2 service call /explore_node/start std_srvs/srv/Trigger
```

---

## License / attribution

BSD, per the upstream package files. Original `m-explore` authors: Jiri Horner et al.;
ROS 2 port: Carlos Alvarez, Juan Galvis. Radiation / battery / dose-budget / mode
extensions: RaMonES project (Brendon Cintas, Florida International University).
