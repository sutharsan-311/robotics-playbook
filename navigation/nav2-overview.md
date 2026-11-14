# Nav2 — Complete Stack Overview

Nav2 is ROS 2's production-grade autonomous navigation framework: a modular collection of lifecycle-managed action servers, costmap layers, and a Behavior Tree orchestrator that replaces the monolithic `move_base` from ROS 1. This article covers every major server, the lifecycle startup sequence, the default Behavior Tree, costmap layer architecture, AMCL, and the most common failure modes.

---

## What it is

Nav2 is not a single algorithm — it is a **system of independent lifecycle-managed servers**, each responsible for one concern. The top-level orchestrator is the **BT Navigator**, which drives navigation by executing a Behavior Tree (XML) that calls the other servers as action clients.

### Full server inventory (Jazzy)

| Server | Package | Role |
|---|---|---|
| `bt_navigator` | `nav2_bt_navigator` | Orchestrates navigation via Behavior Trees |
| `planner_server` | `nav2_planner` | Global path planning (NavFn, Smac Planner) |
| `controller_server` | `nav2_controller` | Local trajectory following, `cmd_vel` output |
| `smoother_server` | `nav2_smoother` | Post-processing path smoothing (Savitzky-Golay, Constrained) |
| `behavior_server` | `nav2_behaviors` | Primitive recoveries: Spin, BackUp, Wait, DriveOnHeading |
| `velocity_smoother` | `nav2_velocity_smoother` | Kinematic smoothing on `cmd_vel` to reduce motor wear |
| `collision_monitor` | `nav2_collision_monitor` | Safety layer: overrides `cmd_vel` on imminent collision |
| `waypoint_follower` | `nav2_waypoint_follower` | Executes ordered `NavigateToPose` sequences |
| `docking_server` | `nav2_docking` | Dock/undock lifecycle with approach controller (Jazzy+) |
| `route_server` | `nav2_route` | Graph-based topological routing (Jazzy+) |
| `map_server` | `nav2_map_server` | Serves static occupancy grid on `/map` |
| `amcl` | `nav2_amcl` | Adaptive Monte Carlo Localization |

*(Source: [nav2_bringup/launch/navigation_launch.py — GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/navigation_launch.py))*

---

## Lifecycle startup sequence

Every server above is a **lifecycle node**. It starts in the `unconfigured` state and must be driven through `configure → activate` before it accepts action requests.

**`nav2_lifecycle_manager`** drives nodes through the lifecycle in a declared order and monitors bond heartbeats to detect crashes. If any node fails to transition, the manager tears down the entire chain in reverse — no partially-active stack.

### Consolidated lifecycle manager (Jazzy — PR #6410)

Before Jazzy, `nav2_bringup` launched two lifecycle managers: one for localization and one for navigation. In Jazzy, `bringup_launch.py` consolidates into a **single** `lifecycle_manager_nav2` that collects node names from all nested launch files via `get_lifecycle_nodes(context)`.

> **Critical:** `ros2 launch nav2_bringup navigation_launch.py` leaves all servers in the `unconfigured` state — the nested launch no longer starts its own lifecycle manager. Use `bringup_launch.py`, or add a `lifecycle_manager` node explicitly with the full node list.

```python
# Minimal lifecycle manager for a custom bring-up
# Source: nav2_bringup/launch/navigation_launch.py (Apache 2.0)
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_nav2',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'autostart': True,
        'node_names': [
            'map_server',
            'amcl',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'velocity_smoother',
            'collision_monitor',
            'bt_navigator',
            'waypoint_follower',
        ],
    }],
)
```

The order in `node_names` matters: the lifecycle manager configures and activates them sequentially from top to bottom, and tears them down in reverse. `map_server` and `amcl` must precede the servers that query the map — placing `bt_navigator` first will fail.

---

## The default Behavior Tree

The `bt_navigator` ships with two default BT XML files:

| BT file | Used by |
|---|---|
| `navigate_to_pose_w_replanning_and_recovery.xml` | `NavigateToPose` |
| `navigate_through_poses_w_replanning_and_recovery.xml` | `NavigateThroughPoses` |

The `navigate_to_pose_w_replanning_and_recovery.xml` tree has two top-level subtrees under a top-level `RecoveryNode`:

1. **Navigation subtree** — `ComputePathToPose` gated behind a `RateController` (1 Hz replanning) feeding into `FollowPath`. Each has a contextual inline recovery (`ClearLocalCostmap` on controller failure, `ClearGlobalCostmap` on planner failure).
2. **Recovery subtree** — triggered only after the navigation subtree exhausts its retries (default 6): `Spin`, `Wait`, `BackUp`, full costmap clear, in a `RoundRobin`.

*(Source: [navigate_to_pose_w_replanning_and_recovery.xml — GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml))*

Switch to a different BT via `bt_navigator` parameters:

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10          # ms — BT tick rate
    default_server_timeout: 20    # ms — timeout for action server connections at startup
    default_nav_to_pose_bt_xml: "$(find-pkg-share my_nav_bringup)/behavior_trees/my_bt.xml"
    default_nav_through_poses_bt_xml: "$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_globally_updated_goal_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_goal_updated_controller_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - nav2_spin_cancel_bt_node
      - nav2_back_up_cancel_bt_node
      - nav2_assisted_teleop_cancel_bt_node
      - nav2_drive_on_heading_cancel_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_to_charging_dock_action_bt_node
```

*(Source: [nav2_bringup/params/nav2_params.yaml — GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml))*

If a BT node type referenced in your XML is absent from `plugin_lib_names`, the BT factory logs "Failed to find action plugin" at startup and the tree loads as an empty structure — the navigator immediately returns `SUCCEEDED` with no motion.

---

## Costmap architecture

Both the `planner_server` (global costmap) and the `controller_server` (local costmap) host their own `nav2_costmap_2d` instance. The costmap subsystem is **plugin-based** — each layer snaps in via the `plugins` parameter list and executes in order: static occupancy from the map → dynamic obstacles from sensors → inflation gradient.

### Standard layer stack

**Global costmap** (full map extent, low update frequency):

```yaml
# Source: nav2_bringup/params/nav2_params.yaml (Apache 2.0)
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true   # see pitfalls
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true
```

**Local costmap** (rolling window around the robot, high update frequency):

```yaml
# Source: nav2_bringup/params/nav2_params.yaml (Apache 2.0)
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true
```

**Key tuning levers:**

`inflation_radius` is the highest-leverage knob in the entire Nav2 stack. Too small: the robot clips wall corners. Too large: the planner fails to find paths through doorways. Start at ~2.5× your robot's inscribed radius and reduce until all doorways are passable with margin.

`cost_scaling_factor` controls how steeply cost decays with distance from obstacles (exponential). Lower values (1.0–2.0) → wide, soft gradients, paths naturally orbit obstacles. Higher values (5.0+) → steep dropoffs, paths move closer to obstacles, plans are more stable in tight corridors.

---

## AMCL

AMCL publishes the `map → odom` transform. Without it (or a SLAM-based alternative), the TF chain is broken and Nav2 cannot function.

```yaml
# Source: nav2_bringup/params/nav2_params.yaml (Apache 2.0)
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2    # rotational noise from rotation
    alpha2: 0.2    # rotational noise from translation
    alpha3: 0.2    # translational noise from translation
    alpha4: 0.2    # translational noise from rotation
    alpha5: 0.2    # strafe noise (omni-directional robots only)
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_model_type: "likelihood_field"
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_z_hit: 0.5
    laser_z_max: 0.05
    laser_z_rand: 0.5
    laser_z_short: 0.05
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2    # min angular displacement before filter update (rad)
    update_min_d: 0.25   # min linear displacement before filter update (m)
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
```

**Tuning guidance:**
- Increase `max_particles` to 5000–10000 on large (>500 m²) or featureless maps (long corridors). The default 2000 spreads too thin and AMCL never fully converges.
- Lower `update_min_d` / `update_min_a` if the robot is slow-moving and localisation drifts while nearly stationary. Raise them if localisation is noisy at speed.
- `transform_tolerance: 1.0` is generous but safe for initial bringup. Tighten to 0.3–0.5 once TF latency is characterised.
- For non-differential robots, replace `nav2_amcl::DifferentialMotionModel` with `nav2_amcl::OmniMotionModel`.

---

## Sending goals — nav2_simple_commander

`nav2_simple_commander` wraps the Nav2 action interfaces behind a clean Python API. It is the recommended way to drive Nav2 from application code.

```python
# Source: nav2_simple_commander/example_nav_to_pose.py
# ros-navigation/navigation2 (Apache 2.0)
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose BEFORE waitUntilNav2Active — AMCL uses it immediately on activate
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()   # blocks until all lifecycle nodes are active

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 17.86
    goal_pose.pose.position.y = -0.77
    goal_pose.pose.orientation.w = 1.0

    task = navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete(task=task):
        feedback = navigator.getFeedback(task=task)
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelTask()

    result = navigator.getResult(task=task)
    if result == TaskResult.SUCCEEDED:
        print('Goal reached!')
    elif result == TaskResult.CANCELED:
        print('Goal canceled.')
    elif result == TaskResult.FAILED:
        print('Goal failed.')
```

For multi-pose missions:
```python
# Navigate through an ordered list of poses (single action, BT handles replanning)
task = navigator.goThroughPoses(poses)   # list[PoseStamped]

# Execute poses sequentially as separate NavigateToPose calls (stop at each waypoint)
task = navigator.followWaypoints(poses)
```

---

## Common pitfalls

**1. Launching `navigation_launch.py` directly in Jazzy leaves servers unconfigured.**
Since PR #6410, nested launch files no longer start their own lifecycle managers. Launching `navigation_launch.py` alone starts the servers but never calls `configure` or `activate`. Any action goal returns `FAILED` immediately. Use `bringup_launch.py`, or add a `lifecycle_manager` node explicitly to your launch file.

**2. `setInitialPose()` after `waitUntilNav2Active()` causes dirty costmap initialisation.**
AMCL starts processing laser scans as soon as it activates. Without a prior initial pose it defaults to the map origin — by the time you call `setInitialPose()`, the global costmap may have already marked large areas as lethal based on the wrong pose. Always call `setInitialPose()` before `waitUntilNav2Active()`.

**3. Inconsistent `use_sim_time` across Nav2 nodes.**
Every server in the lifecycle list, plus AMCL, the TF publisher, and your autonomy node, must agree on `use_sim_time`. A single node on wall clock time produces TF lookups that fail with "Lookup would require extrapolation into the past" — which Nav2 logs as a goal failure, not a time configuration warning. Push `use_sim_time` into every node's `parameters` list via `LaunchConfiguration`.

**4. Missing `odom` frame — global costmap uses `map`, local costmap uses `odom`.**
The global costmap's `global_frame` is `map`; the local costmap's is `odom`. Nav2 specifically queries the robot's pose in the `odom` frame for the controller and in the `map` frame for the planner. If your localizer publishes `map → base_link` directly with no odometry frame, the controller server fails with "Could not get robot pose" even though global planning succeeds. Always maintain a `map → odom → base_link` chain.

**5. `inflation_radius` larger than half the doorway width blocks entrances.**
If `inflation_radius` is 0.55 m and your doorway is 0.9 m wide, the inflated obstacles from both door jambs overlap — the planner sees no free cell through the opening and returns `NO_VALID_PATH`. Reduce `inflation_radius` until `2 × inflation_radius < doorway_width`, then compensate with the controller's collision avoidance threshold or a narrower footprint for the planning phase.

**6. Static layer without `map_subscribe_transient_local: true`.**
The `/map` topic is published with `transient_local` durability — the map server holds the last message for late-joining subscribers. If `map_subscribe_transient_local` is not set to `true` in the static layer config, the layer waits for a fresh map message that never arrives (the message was published before the costmap activated). The costmap initialises as fully unknown and no plan can be computed. This is the number-one cause of "costmap is empty" on first run.

**7. `max_particles` too low on large or featureless maps.**
The default `max_particles: 2000` is calibrated for TurtleBot3 in a ~20 m² test environment. On a 50×50 m warehouse or on maps with long identical corridors, 2000 particles spread too thin and AMCL never converges — `amcl_pose` covariance stays large and the robot drifts off the global plan. Raise to 5000–10000; modern Intel CPUs handle 10,000 particles easily.

**8. `plugin_lib_names` incomplete for a custom BT.**
Any BT node type in your XML that is absent from `plugin_lib_names` causes the BT factory to log "Failed to find action plugin" at startup and the tree loads as empty — the navigator immediately returns `SUCCEEDED` with no motion. After swapping BT XMLs, diff your `plugin_lib_names` list against the node types used in the new XML.

---

## Further reading

- [Nav2 Concepts — navigation.ros.org](https://navigation.ros.org/concepts/index.html) — authoritative overview of server architecture, action interfaces, TF requirements, sensor setup
- [Configuring the BT Navigator — navigation.ros.org](https://navigation.ros.org/configuration/packages/configuring-bt-navigator.html) — all bt_navigator parameters: frame IDs, BT loop duration, plugin_lib_names, default BT XML paths
- [Detailed Behavior Tree Walkthrough — navigation.ros.org](https://navigation.ros.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html) — node-by-node explanation of the default NavigateToPose BT including the recovery subtree logic
- [Lifecycle Manager — navigation.ros.org](https://navigation.ros.org/configuration/packages/configuring-lifecycle.html) — node_names ordering, bond_timeout, autostart, and failure handling
- [nav2_bringup/params/nav2_params.yaml — GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml) — the official reference configuration for every server; diff your params against this when debugging
- [navigate_to_pose_w_replanning_and_recovery.xml — GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml) — the actual default BT XML: read this before writing a custom tree
- [nav2_simple_commander API — navigation.ros.org](https://navigation.ros.org/commander_api/index.html) — full Python API reference: goToPose, goThroughPoses, followWaypoints, cancelTask, getResult, getFeedback

---

*2026-11-14 | ROS2 version: Jazzy / Humble*
