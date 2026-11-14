# Nav2 Controllers — Following a Path

The controller server is where a planned path becomes actual wheel commands. It runs at high frequency, ingests the global path, and outputs `geometry_msgs/TwistStamped` on `/cmd_vel`. Choosing and tuning the right controller plugin is one of the highest-leverage decisions in a Nav2 deployment.

## What it is

The `controller_server` node hosts one or more **controller plugins** — each implementing `nav2_core::Controller`. When the BT navigator calls `FollowPath`, the server selects the right plugin by name, feeds it the path, and calls `computeVelocityCommands()` at `controller_frequency` Hz until the goal checker signals success or a progress checker fires a failure.

Nav2 ships three production-grade controllers:

| Plugin | Type | Best for |
|---|---|---|
| `nav2_dwb_controller::DWBLocalPlanner` | Reactive sampling (DWA variant) | Omni-directional, heavily cluttered spaces |
| `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | Geometric (pure pursuit + speed regulation) | Exact path following on diff/ackermann/legged |
| `nav2_mppi_controller::MPPIController` | Stochastic MPC | Complex cost landscapes, high dynamic obstacle density |

You can load multiple plugins and name them separately — useful when different behaviour trees call different controllers for different task phases.

## How it works

**Regulated Pure Pursuit (RPP)** is the most common starting point. The classic pure pursuit algorithm finds a *lookahead point* on the path at distance $L$ ahead of the robot, computes the arc curvature to reach it, and derives angular velocity. RPP adds two regulation layers:

1. **Curvature-based speed reduction** — linear velocity is scaled down when the path curves sharply, proportional to the signed curvature of the arc.
2. **Collision-time reduction** — if the forward projection of the robot at current speed would collide with a costmap obstacle within `max_allowed_time_to_collision_up_to_carrot` seconds, velocity is reduced to open the time window.

A minimal `controller_server` configuration using RPP (from the [official Nav2 README](https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md)):

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      use_velocity_scaled_lookahead_dist: false
      use_regulated_linear_velocity_scaling: true
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_rotate_to_heading: true
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
```

**DWB** works differently: it samples a discrete set of `(v, ω)` pairs within the robot's dynamic window, rolls out short arcs for each, and scores them through a stack of *critic plugins* (path alignment, goal heading, obstacle proximity, etc.). You tune it by adjusting critic weights rather than geometric parameters.

**MPPI** generates thousands of randomly noised control sequences, rolls them forward in parallel using GPU or vectorised CPU ops, weights them by a cost function, and blends into an optimal command. It is the most capable but also the most compute-hungry; it needs consistent high-rate execution (≥30 Hz) or trajectory quality degrades.

## Common pitfalls

**RPP does not avoid dynamic obstacles.** It is a purely geometric controller — it tracks the arc to the lookahead point regardless of what appears between it and the robot. If an obstacle blocks the path, RPP returns failure, triggering a replan. Set `controller_frequency` high enough and `recovery_behaviors` correctly so replanning is fast; don't rely on RPP to dodge moving objects.

**Lookahead distance is a stability dial, not just a speed knob.** `lookahead_dist` too small → oscillation and aggressive heading corrections. Too large → the robot cuts corners and drifts wide of the path in tight corridors. A starting heuristic: set it to roughly 2× your robot's wheelbase for differential drives, then tune from there using the `/local_plan` visualisation topic.

**`transform_tolerance` must exceed your worst-case TF latency.** The RPP controller (and most others) looks up the robot's pose in the costmap frame at the current timestamp. If your TF pipeline has occasional latency spikes above `transform_tolerance`, the controller will throw `"Could not get robot pose"` and abort the action. Set `transform_tolerance` to 0.2–0.3 s in simulation or on hardware with network-bridged TF, not the default 0.1 s.

## Further reading

- [Controller Server configuration reference — navigation.ros.org](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
- [Regulated Pure Pursuit configuration — navigation.ros.org](https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html)
- [Nav2 controller plugin list (all built-ins) — docs.nav2.org](https://docs.nav2.org/plugins/)

---
*2026-11-14 | ROS2 version: Jazzy/Humble*
