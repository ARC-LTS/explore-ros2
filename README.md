# explore-ros2 — Radiation-Aware Frontier Exploration

ROS 2 port of [m-explore](https://github.com/hrnr/m-explore) extended with radiation-aware and battery-aware autonomous exploration for hazardous environment robotics. This fork is maintained as a component of the RaMonES (Radiation Monitoring and Exploration System) stack.

Tested on Eloquent, Dashing, Foxy, and Galactic distros.

---

## Contents

1. [Packages](#packages)
2. [Changes from upstream](#changes-from-upstream)
   - [Radiation-aware exploration](#radiation-aware-exploration)
   - [Battery-aware navigation](#battery-aware-navigation)
   - [Dijkstra-based path distance](#dijkstra-based-path-distance)
   - [Enhanced goal management](#enhanced-goal-management)
   - [Service-based control API](#service-based-control-api)
   - [Visualization enhancements](#visualization-enhancements)
   - [Verification and recovery](#verification-and-recovery)
3. [Parameters reference](#parameters-reference)
4. [Building](#building)
5. [Running](#running)
6. [Multirobot map merge](#multirobot-map-merge)

---

## Packages

| Package | Description |
|---|---|
| `explore_lite` | Greedy frontier-based exploration node with radiation and battery awareness |
| `multirobot_map_merge` | Merges occupancy maps from multiple robots into a single global map |

---

## Changes from upstream

The upstream m-explore-ros2 port provides a standard greedy frontier exploration algorithm. This fork introduces substantial modifications to support autonomous operation in radiation-contaminated environments.

### Radiation-aware exploration

The frontier cost function now incorporates radiation data from a heatmap topic (`/rad_heatmap/raw_mean_map`). Each frontier candidate is penalized according to the estimated radiation dose it would expose the robot to, discouraging navigation toward high-dose zones.

**Cumulative dose tracking** integrates the dose rate over time. When the accumulated dose approaches `max_radiation_dose`, the robot initiates a return-to-origin sequence.

**Acute dose rate evacuation** provides a fast-path safety response: if the instantaneous dose rate exceeds `acute_dose_rate_mrem_hr` (default 10 mR/hr), exploration is immediately interrupted and the robot retreats.

**Radiation-seeking mode** inverts the cost penalty so that frontiers near higher radiation gradients are prioritized. This is useful for locating and characterizing radiation sources. The mode is toggled at runtime via a service call (see [Service-based control API](#service-based-control-api)).

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `radiation_topic` | `rad` | Topic for instantaneous dose rate readings |
| `heatmap_topic` | `/rad_heatmap/raw_mean_map` | GP heatmap image used for frontier cost |
| `max_radiation_dose` | `100.0` mR | Cumulative dose limit triggering return |
| `critical_dose_mrem_hr` | `2.0` mR/hr | Dose rate at which frontier cost penalty activates |
| `acute_dose_rate_mrem_hr` | `10.0` mR/hr | Dose rate triggering immediate evacuation |
| `dose_budget_scale` | `10.0` | Weight of dose budget penalty in cost function |
| `safe_dose_threshold_mrem_hr` | `1.0` mR/hr | Dose rate below which an area is considered safe |
| `radiation_scale` | `1.0` | Multiplier on heatmap-derived cost contribution |
| `radiation_gradient_scale` | `1.0` | Multiplier on gradient-based cost contribution |

---

### Battery-aware navigation

The node subscribes to a `sensor_msgs/BatteryState` topic and estimates whether the robot has sufficient charge to reach each frontier candidate and return to the origin. Frontiers that are estimated to drain the battery beyond `min_battery_percentage` are penalized in the cost function.

When the battery state reaches the threshold, the node stops exploration and autonomously returns to the initial pose.

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `battery_topic` | `/battery_state` | Battery state topic |
| `energy_per_meter` | `0.1` | Estimated energy consumed per metre of travel |
| `battery_safety_margin` | `0.1` | Reserve fraction kept for the return trip |
| `battery_penalty_scale` | `1.0` | Weight of battery penalty in cost function |
| `min_battery_percentage` | `0.2` | Battery fraction below which return is triggered |
| `max_battery_used` | `0.8` | Maximum allowable fraction of total charge used |

---

### Dijkstra-based path distance

Upstream `FrontierSearch` ranked frontiers using straight-line (Euclidean) distance, which ignores obstacles and narrow passages. This fork replaces that with an 8-connected Dijkstra search over the costmap, computing actual traversal cost to each frontier centroid. Diagonal steps are weighted by `√2`. This produces more accurate rankings in cluttered environments and prevents the robot from repeatedly targeting frontiers that are geometrically close but navigationally distant.

---

### Enhanced goal management

**Hysteresis bonus**: the current navigation goal receives a cost reduction proportional to `gain_scale`. This prevents thrashing when two frontiers have similar scores and reduces unnecessary goal switches.

**Goal queuing**: the node builds an ordered queue of frontier goals and steps through it on failure or goal completion, avoiding repeated re-computation of the full frontier set for every navigation step.

**Replanning threshold**: if the robot is already within `min_goal_distance_for_replanning` metres of the current goal, no new goal is dispatched. This prevents micro-replanning oscillations.

**Goal blacklisting**: goals that the navigation stack consistently fails to reach are blacklisted and removed from future consideration.

Key parameters:

| Parameter | Default | Description |
|---|---|---|
| `min_goal_distance_for_replanning` | `0.75` m | Distance below which replanning is suppressed |
| `progress_timeout` | `30.0` s | Time without progress before goal is abandoned |
| `no_frontier_retry` | `3` | Consecutive no-frontier cycles before finishing |

---

### Service-based control API

In addition to the upstream `explore/resume` topic, this fork exposes three services for programmatic control:

| Service | Type | Description |
|---|---|---|
| `~/start` | `std_srvs/Trigger` | Start or resume frontier exploration |
| `~/stop` | `std_srvs/Trigger` | Pause exploration and cancel current navigation goal |
| `~/return_to_initial_pose` | `std_srvs/Trigger` | Navigate back to the pose recorded at startup |
| `~/set_radiation_seeking` | `std_srvs/SetBool` | Enable (`true`) or disable (`false`) radiation-seeking mode |

The `~/mode` topic publishes the current exploration state as a `std_msgs/String` (`exploring`, `returning`, `stopped`, etc.).

---

### Visualization enhancements

- **Frontier markers** (`explore/frontiers`): each frontier is rendered as a colored marker. The colour reflects the frontier's computed cost, allowing operators to inspect the cost function directly in RViz2.
- **Explored area grid** (`explore/explored_area`): publishes a binary occupancy grid highlighting cells confirmed as traversed.
- **Heatmap overlay**: when `heatmap_topic` is configured, radiation intensity is overlaid on the frontier marker colours.

---

### Verification and recovery

Before marking an area as fully explored, the node runs up to `max_verification_attempts` navigation attempts to frontiers in the region. This reduces premature termination due to transient navigation failures. After exhausting verification attempts, the frontier is blacklisted.

| Parameter | Default | Description |
|---|---|---|
| `max_verification_attempts` | `5` | Attempts before a frontier is blacklisted |
| `return_to_init` | `true` | Return to start pose when exploration completes |
| `auto_start` | `false` | Begin exploring immediately on node startup |

---

## Parameters reference

All parameters are set in `explore/config/params.yaml`. The full set:

```yaml
explore:
  ros__parameters:
    # Frame and topic configuration
    robot_base_frame: base_link
    costmap_topic: /global_costmap/costmap
    costmap_updates_topic: /global_costmap/costmap_updates

    # Frontier cost weights
    potential_scale: 10.0
    orientation_scale: 2.0
    gain_scale: 5.0
    min_frontier_size: 0.25
    min_frontier_clearance: 0.0

    # Planner
    planner_frequency: 1.0
    progress_timeout: 30.0
    min_goal_distance_for_replanning: 0.75
    no_frontier_retry: 3

    # Battery
    battery_topic: /battery_state
    energy_per_meter: 0.1
    battery_safety_margin: 0.1
    battery_penalty_scale: 1.0
    min_battery_percentage: 0.2
    max_battery_used: 0.8

    # Radiation
    radiation_topic: rad
    heatmap_topic: /rad_heatmap/raw_mean_map
    max_radiation_dose: 100.0
    critical_dose_mrem_hr: 2.0
    acute_dose_rate_mrem_hr: 10.0
    dose_budget_scale: 10.0
    safe_dose_threshold_mrem_hr: 1.0
    radiation_scale: 1.0
    radiation_gradient_scale: 1.0

    # Verification and control
    max_verification_attempts: 5
    return_to_init: true
    auto_start: false
    visualize: true
```

---

## Building

Build as a standard colcon package. Use rosdep to resolve dependencies:

```bash
cd <ros2_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select explore_lite multirobot_map_merge
```

---

## Running

### With the RaMonES stack

The recommended way to run this package is via the `rad_launch` orchestration launch file, which starts the full simulation, SLAM, Nav2, radiation heatmapping, and exploration stack in the correct order:

```bash
ros2 launch rad_launch rad_launch_sim_launch.py
```

The `explore_lite` node starts 25 seconds into the sequence, after Nav2 and the GP heatmap pipeline have initialised.

### Standalone

```bash
ros2 launch explore_lite explore.launch.py
```

Or with an explicit params file:

```bash
ros2 run explore_lite explore --ros-args --params-file <path>/explore/config/params.yaml
```

### With Nav2 and SLAM (TB3 demo)

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
ros2 launch explore_lite explore.launch.py
```

Add the `explore/frontiers` marker topic in RViz2 to visualise frontier candidates and the cost-weighted selection.

---

## Multirobot map merge

The `multirobot_map_merge` package is a direct ROS 2 port of the upstream m-explore map merging node. No algorithmic changes have been made relative to the upstream port.

### Known initial poses

Produces the best merge quality. Launch the full simulation stack and then:

```bash
ros2 launch multirobot_map_merge map_merge.launch.py known_init_poses:=True
```

Initial poses for each robot are set as parameters under `/<robot_ns>/map_merge/init_pose_x`, `init_pose_y`, etc.

### Unknown initial poses

Works best when robots start within ~3 metres of each other. Uses AKAZE/ORB feature matching on the occupancy grid images to estimate relative transforms:

```bash
ros2 launch multirobot_map_merge map_merge.launch.py known_init_poses:=False
```

### TB3 multi-robot demo

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True
ros2 launch multirobot_map_merge map_merge.launch.py
```

Visualise the merged map:

```bash
rviz2 -d <ros2_ws>/src/explore-ros2/map_merge/launch/map_merge.rviz
```

---

## License

Packages are licensed under the BSD license. See respective package files for details.

Original m-explore authors: Jiri Horner et al.  
ROS 2 port authors: Carlos Alvarez, Juan Galvis.  
Radiation and battery extensions: RaMonES project contributors (Brendon Cintas, Florida International University).
