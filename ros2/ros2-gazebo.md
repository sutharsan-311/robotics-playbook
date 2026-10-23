# ROS2 Simulation with Gazebo

Gazebo (formerly Ignition Gazebo, now "Gz Sim") is the standard physics simulator for ROS2 robotics development — it lets you validate perception, control, and navigation algorithms without hardware, using the same message interfaces your real robot uses.

## What it is

Modern Gazebo (Gz Sim) is a ground-up rewrite of Gazebo Classic, which reached end-of-life in January 2025. The current generation is modular, plugin-based, and communicates via **Gazebo Transport** — a separate pub/sub system from ROS2's DDS. The `ros_gz` package family bridges the two.

**Version pairings you should use:**

| ROS 2 Distro | Gazebo Version | Support |
|---|---|---|
| Jazzy Jalisco (LTS) | Harmonic (LTS) | ✅ Recommended |
| Humble Hawksbill (LTS) | Fortress | ✅ Default |
| Humble | Harmonic | ⚠️ Possible, unofficial binaries |

For new projects on Ubuntu 24.04: **Jazzy + Harmonic** is the correct pairing. Install via:

```bash
sudo apt install ros-jazzy-ros-gz
```

This pulls in `ros_gz_bridge`, `ros_gz_sim`, and `ros_gz_interfaces`.

## How it works

The three key packages:

- **`ros_gz_sim`** — launches Gazebo and spawns entities
- **`ros_gz_bridge`** — translates messages between ROS2 topics and Gazebo Transport topics
- **`ros_gz_interfaces`** — shared message type definitions

### Launching Gazebo from ROS2

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf -r"
```

The `-r` flag starts simulation running immediately. Without it, you must press Play in the GUI.

### Spawning a robot

From a launch file, use `ros_gz_sim`'s `create` node to inject a robot URDF from the `robot_state_publisher`'s `/robot_description` topic:

```python
Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'my_robot',
        '-topic', '/robot_description',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.01',   # spawn 1 cm above ground — see pitfalls
    ],
    output='screen',
)
```

### Bridging topics

The `parameter_bridge` node translates between ROS2 and Gazebo Transport. The syntax for each bridged topic is:

```
/TOPIC@ROS_MSG_TYPE[gz_direction]gz.msgs.GzType
```

Direction indicators: `@` = bidirectional, `[` = Gz→ROS, `]` = ROS→Gz.

A concrete example bridging `cmd_vel` (ROS→Gz) and odometry (Gz→ROS):

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

For anything beyond a couple of topics, use a YAML config file instead:

```yaml
# bridge.yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/my_robot/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/odom"
  gz_topic_name: "/model/my_robot/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```

## Common pitfalls

**1. `use_sim_time` is per-node, not global.**  
Unlike ROS1, there is no global `use_sim_time` parameter in ROS2. You must set it on *every* node that needs simulated time. Missing even one — especially `robot_state_publisher` or a controller — produces transform extrapolation errors that are hard to trace. Set it in your launch file:
```python
parameters=[{'use_sim_time': True}]
```

**2. Spawning at z=0 triggers physics explosions.**  
If a wheeled robot's collision geometry is placed exactly at the ground plane (z=0.0), the physics engine detects an immediate contact penetration and applies enormous corrective forces, launching the robot into the sky. Always spawn at a small positive z offset (`z=0.01` is the conventional value).

**3. URDF→SDF conversion drops sensor definitions on joints.**  
Gazebo converts your URDF to SDF internally, but sensors declared on a `<joint>` element are silently ignored — only sensors on `<link>` elements survive the conversion. If a sensor isn't appearing in simulation, check which element it's parented to in your URDF and move it to a link.

## Further reading

- [ROS 2 integration overview — Gazebo Harmonic docs](https://gazebosim.org/docs/harmonic/ros2_overview/)
- [Use ROS 2 to interact with Gazebo — Gazebo Harmonic docs](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [gazebosim/ros_gz — source, README, and compatibility branches](https://github.com/gazebosim/ros_gz/blob/ros2/README.md)

---
*2026-10-23 | ROS2 version: Jazzy / Humble — Gazebo Harmonic / Fortress*
