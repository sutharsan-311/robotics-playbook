# Nav2 Planners — Path Planning Algorithms

Nav2's Planner Server is the global path-planning layer of your navigation stack. Choosing the wrong planner for your robot kinematics is one of the fastest ways to produce physically infeasible paths — understanding the available algorithms and when each applies is essential.

## What it is

The `nav2_planner` server hosts the **global costmap** and routes `ComputePathToPose` / `ComputePathThroughPoses` action requests to one or more registered planner plugins. Each plugin is a shared library implementing the `nav2_core::GlobalPlanner` interface. You can run multiple planners simultaneously and select between them at runtime (useful when you have both a diff-drive base and a manipulator arm that needs Cartesian path planning in the same system).

Nav2 ships four first-party planners:

| Plugin class | Algorithm | Robot type |
|---|---|---|
| `nav2_navfn_planner::NavfnPlanner` | Dijkstra or A* on a 2D grid | Circular/diff-drive |
| `nav2_smac_planner::SmacPlanner2D` | Cost-aware 2D A* | Circular diff-drive / omni |
| `nav2_smac_planner::SmacPlannerHybrid` | Hybrid-A* with kinematic constraints | Ackermann / car-like / legged |
| `nav2_smac_planner::SmacPlannerLattice` | State Lattice (precomputed motion primitives) | Any — requires primitive files |
| `nav2_theta_star_planner::ThetaStarPlanner` | Any-angle Theta* | Circular diff-drive / omni |

## How it works

**NavFn** runs Dijkstra (default) or A* across the full inflated costmap. It is fast but treats the robot as a point (or uniform circle), so paths can swing wide and disregard the actual footprint in narrow corridors.

**SmacPlanner2D** replaces NavFn's sweep with a cost-aware A* that properly weights traversal through high-cost cells. Paths hug obstacles less aggressively than NavFn.

**SmacPlannerHybrid** adds SE(2) state space — position *and* heading — with Dubin or Reeds-Shepp motion models. This makes it the correct choice any time minimum turning radius matters (delivery robots, forklifts, any Ackermann platform). It is more computationally expensive than the 2D planners.

**ThetaStar** relaxes A*'s constraint that path segments must follow grid edges, drawing straight lines between nodes at any angle. This produces shorter, more natural-looking paths for holonomic robots without the overhead of SE(2) search.

### Configuration example (from official Nav2 docs)

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      downsample_costmap: false
      downsampling_factor: 1
      tolerance: 0.25
      allow_unknown: true
      max_iterations: 1000000
      max_planning_time: 5.0
      motion_model_for_search: "DUBIN"       # or "REEDS_SHEPP" for reverse
      angle_quantization_bins: 72            # heading resolution
      analytic_expansion_ratio: 3.5
      minimum_turning_radius: 0.40           # metres — match your platform
```

*Source: [Nav2 Smac Hybrid-A* Planner docs](https://navigation.ros.org/configuration/packages/smac/configuring-smac-hybrid.html)*

To run multiple planners in parallel (e.g. NavFn for quick replanning and Hybrid-A* for precise docking):

```yaml
planner_plugins: ["GridBased", "Precise"]
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  use_astar: true
Precise:
  plugin: "nav2_smac_planner::SmacPlannerHybrid"
  minimum_turning_radius: 0.40
```

The action goal's `planner_id` field selects which plugin handles the request.

## Common pitfalls

1. **NavFn on non-circular robots.** NavFn approximates the robot as circular using the `robot_radius` parameter of the global costmap. In tight corridors the inflation layer does the collision-avoidance work, so if your costmap inflation radius is too small, NavFn will plan paths the actual footprint cannot physically follow.

2. **Wrong motion model for your platform.** Using `SmacPlannerHybrid` with `REEDS_SHEPP` on a robot that cannot reverse will produce paths the controller rejects. Set `DUBIN` for forward-only vehicles and verify `minimum_turning_radius` against your actual chassis spec — a value that is too small causes the path to be kinematically infeasible.

3. **Planning timeout in large or cluttered maps.** `max_planning_time` for Hybrid-A* defaults to 5 s. In highly cluttered environments with a large `max_iterations` budget, the planner can hit this limit and return a partial path (if `allow_partial_planning: true`) or fail. Tune `downsample_costmap: true` with `downsampling_factor: 2` to trade resolution for speed when the global map is large.

## Further reading

- [Nav2 Planner Server configuration reference](https://docs.nav2.org/configuration/packages/configuring-planner-server.html)
- [Smac Planner README — algorithm details & all parameters](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md)
- [Nav2 Plugin index — full list of planner, controller, and smoother plugins](https://navigation.ros.org/plugins/index.html)

---
*2026-11-13 | ROS2 version: Jazzy / Humble*
