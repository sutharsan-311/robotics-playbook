# Nav2 Costmaps — Local and Global

Nav2 uses two costmaps to separate concerns: a global costmap for full-map path planning and a local costmap for real-time obstacle avoidance. Getting their configuration right is what separates a robot that navigates cleanly from one that freezes, clips corners, or refuses to plan.

## What it is

A costmap is a 2D grid where each cell holds a cost value (0–255) representing traversal risk. Nav2 maintains two independent instances:

| | **Global Costmap** | **Local Costmap** |
|---|---|---|
| Frame | `map` | `odom` |
| Rolling window | No (fixed) | Yes |
| Typical size | Full map extent | 3–5 m around robot |
| Update rate | 1–5 Hz | 5–10 Hz |
| Primary use | Path planning (planner server) | Collision avoidance (controller server) |

Both costmaps are built from **layers** — plugins that each contribute obstacle information — stacked and merged in order. The standard layer set is:

- **`StaticLayer`** — imports the occupancy grid published to `/map` (from SLAM or map_server)
- **`ObstacleLayer`** — marks and clears obstacles from `LaserScan` or `PointCloud2`; uses raycasting to clear free space
- **`VoxelLayer`** — 3D variant of ObstacleLayer using pointclouds; projects down to 2D
- **`InflationLayer`** — expands every lethal cell outward with exponentially decaying cost

Cost encoding: `0` = free, `253` = inscribed (robot center is touching obstacle), `254` = lethal, `255` = unknown. The inflation formula is:

```
cost = 253 × exp(−cost_scaling_factor × (d − inscribed_radius))
```

where `d` is the distance in metres from the nearest lethal cell.

## How it works

The `costmap_2d` node runs both costmaps as lifecycle nodes. Each costmap processes its layer stack on every `update_frequency` tick, then the resulting grid is published for the planner/controller servers to consume.

Below is a representative `nav2_params.yaml` configuration (derived from the official [nav2_bringup params](https://github.com/ros-navigation/navigation2/tree/main/nav2_bringup)):

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22          # or use footprint: for non-circular robots
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: True
          clearing: True
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: True
          clearing: True
          raytrace_max_range: 3.0
          obstacle_max_range: 2.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

Note the double-nesting (`global_costmap.global_costmap.ros__parameters`) — this is the actual node/namespace structure used by nav2_bringup and is a frequent source of confusion.

The local costmap omits `static_layer` because it has no use for a fixed map background; it only cares about what sensors see right now. It also sets `rolling_window: true` and specifies an explicit `width`/`height` in metres.

## Common pitfalls

**1. InflationLayer is not last in the plugin list.**
Layers are composited in order. If `inflation_layer` appears before `obstacle_layer`, newly marked obstacles won't be inflated — the robot detects the wall but plans directly against it. Always put `inflation_layer` last.

**2. `cost_scaling_factor` is too high.**
A high value (e.g., 10.0) makes the inflation cost drop almost instantly to 0 just a few cells from an obstacle. The global planner sees near-zero cost near walls and happily routes through them. A value of 2.5–4.0 gives a smooth gradient that discourages close proximity without blocking all narrow corridors.

**3. `track_unknown_space: false` on the global costmap.**
With this off, every unseen cell is treated as free (cost 0). The planner will cheerfully send the robot into unexplored space. Set it to `true` so the planner treats unknowns as obstacles-until-proven-free, especially in SLAM-driven deployments where the map arrives incrementally.

**Bonus: TF timing issues with the obstacle layer.**
If your sensor's TF is not being published at high enough frequency, the `ObstacleLayer` will silently skip updates — obstacles appear only intermittently on the costmap. Confirm with `ros2 run tf2_tools view_frames` and check your sensor driver's TF publish rate.

## Further reading

- [Costmap 2D — Nav2 Configuration Reference](https://navigation.ros.org/configuration/packages/configuring-costmaps.html) — official parameter reference with all layer options
- [nav2_costmap_2d source — ros-navigation/navigation2](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d) — plugin headers and default values in `nav2_costmap_2d_ros.cpp`
- [Writing a New Costmap2D Plugin — Nav2 Docs](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) — if you need to add a custom semantic or gradient layer

---

*2026-11-11 | ROS2 version: Jazzy / Humble*
