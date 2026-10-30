# Nav2 — Complete Stack Overview

Nav2 is ROS 2's production-grade autonomous navigation framework: a modular collection of action servers, costmap layers, and a Behavior Tree orchestrator that replaces the monolithic `move_base` from ROS 1.

---

## What it is

Nav2 is not a single algorithm — it is a **system of independent lifecycle-managed servers**, each responsible for one concern. The top-level entry point is the **BT Navigator**, which drives navigation by executing a Behavior Tree (XML) that calls the other servers as BT actions.

The seven core servers:

| Server | Package | Role |
|---|---|---|
| `bt_navigator` | `nav2_bt_navigator` | Orchestrates navigation via Behavior Trees |
| `planner_server` | `nav2_planner` | Global path planning (e.g. NavFn, Smac Planner) |
| `controller_server` | `nav2_controller` | Local trajectory following & velocity commands |
| `smoother_server` | `nav2_smoother` | Post-processing path smoothing |
| `behavior_server` | `nav2_behaviors` | Primitive recoveries (spin, back-up, wait) |
| `map_server` | `nav2_map_server` | Serves static occupancy grid |
| `amcl` | `nav2_amcl` | Adaptive Monte Carlo Localization |

All servers (except `map_server` and `amcl` in static-map mode) are **lifecycle nodes**. The `nav2_lifecycle_manager` brings them up and tears them down in dependency order — a startup failure in `map_server` will abort the whole chain cleanly rather than leaving half-initialised nodes running.

The costmap subsystem (`nav2_costmap_2d`) is embedded inside both `planner_server` (global costmap) and `controller_server` (local costmap). It is plugin-based: inflation layer, obstacle layer, voxel layer, and SLAM-integrated layers all snap in via the `plugins` parameter.

---

## How it works

Navigation requests arrive via two action interfaces:
- `NavigateToPose` — single goal
- `NavigateThroughPoses` — ordered waypoint list

The `bt_navigator` receives the goal, loads the configured BT XML file, and ticks it. The BT calls `ComputePathToPose` on the planner server, then `FollowPath` on the controller server. If the controller stalls, recovery subtrees invoke the behavior server before retrying.

The `nav2_simple_commander` package wraps all of this behind a clean Python API. This is the recommended way to drive Nav2 programmatically — from an autonomy application, a mission manager, or a test harness. The following is taken directly from the official example at `nav2_simple_commander/example_nav_to_pose.py`:

```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose (must be called before waitUntilNav2Active)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 17.86
    goal_pose.pose.position.y = -0.77
    goal_pose.pose.orientation.w = 1.0

    go_to_pose_task = navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete(task=go_to_pose_task):
        feedback = navigator.getFeedback(task=go_to_pose_task)
        # your application logic here

    result = navigator.getResult(task=go_to_pose_task)
    if result == TaskResult.SUCCEEDED:
        print('Goal reached!')
    elif result == TaskResult.CANCELED:
        print('Goal canceled.')
    elif result == TaskResult.FAILED:
        print('Goal failed.')
```

*Source: [ros-navigation/navigation2 on GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_nav_to_pose.py)*

The `waitUntilNav2Active()` call polls the lifecycle manager and blocks until all servers are in the `active` state — critical when your autonomy node starts before Nav2 finishes configuring.

---

## Common pitfalls

**1. Setting initial pose after `waitUntilNav2Active()`**
Calling `setInitialPose()` after Nav2 activates causes the costmap to initialise at the map origin (0, 0) and mark large areas as unknown or lethal before AMCL corrects itself. Always call `setInitialPose()` first, then `waitUntilNav2Active()`.

**2. Inconsistent `use_sim_time` across nodes**
All Nav2 servers, your autonomy node, AMCL, and the TF broadcaster must agree on `use_sim_time`. A single node running on wall-clock time while the rest uses `/clock` causes TF lookups to time out or costmaps to stop updating silently. Set it globally in your launch file via `LaunchConfiguration`.

**3. Global costmap frame vs. odom frame confusion**
Nav2 expects a full TF chain: `map → odom → base_link`. If your localizer publishes `map → base_link` directly (no odometry frame), the controller server will fail with "Could not get robot pose" because it specifically queries the local costmap frame (`odom`). Always maintain a separate `odom` frame even in simulation.

---

## Further reading

- [Nav2 Concepts — navigation.ros.org](https://navigation.ros.org/concepts/index.html)
- [Nav2 Lifecycle Manager — docs.ros.org (Jazzy)](https://docs.ros.org/en/jazzy/p/nav2_lifecycle_manager)
- [nav2_simple_commander package — index.ros.org](https://index.ros.org/p/nav2_simple_commander/)

---

*2026-10-30 | ROS2 version: Jazzy / Humble*
