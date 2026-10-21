# ROS2 Visualization with RViz2

RViz2 is the primary 3D visualization tool for ROS 2 — the canonical way to inspect sensor data, TF trees, robot models, and costmaps during development and debugging.

## What it is

RViz2 is a port of RViz to ROS 2, shipping as the `rviz2` package under the `ros-visualization` umbrella. It runs as a standalone Qt-based GUI that subscribes to ROS 2 topics and renders their data in a 3D scene. Every visualization unit — a laser scan, a robot model, a point cloud, an arrow marker — is a *Display*, each independently configured and toggled.

RViz2 is **not** a simulator; it only reads and renders live topic data. Think of it as an oscilloscope for your robot: it shows what is happening in the system right now.

Key built-in display types relevant to most projects:

| Display | Message type | Typical use |
|---|---|---|
| RobotModel | `/robot_description` (URDF) | Visualize kinematic state |
| TF | `tf2_msgs/TFMessage` | Inspect the full transform tree |
| LaserScan | `sensor_msgs/LaserScan` | 2D lidar returns |
| PointCloud2 | `sensor_msgs/PointCloud2` | 3D depth / lidar clouds |
| MarkerArray | `visualization_msgs/MarkerArray` | Custom overlays, arrows, spheres |
| Map | `nav_msgs/OccupancyGrid` | SLAM / nav costmaps |
| Path | `nav_msgs/Path` | Planned or executed trajectories |

## How it works

Every display is anchored to a **Fixed Frame** — the coordinate frame that the 3D scene is built around (typically `map`, `odom`, or `base_link`). When a message arrives, RViz2 looks up the TF transform from `header.frame_id` into the Fixed Frame and places the data there. No valid transform → no rendering.

Configuration is persisted in `.rviz` YAML files and should live inside your package's `rviz/` directory so they travel with the source tree. The standard pattern for launching RViz2 from a Python launch file, drawn directly from the Navigation2 `nav2_bringup` package:

```python
# Source: https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/rviz_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('rviz_config_file', default_value=''),
        start_rviz_cmd,
    ])
```

The `-d` argument loads a saved `.rviz` config. The `use_sim_time` parameter is critical when running with a bag or simulator — without it, timestamps from the clock topic won't match RViz2's wall-clock expectations.

You can also launch RViz2 headlessly to verify topic output or run it inside a CI pipeline by setting the `DISPLAY` environment variable (e.g. with Xvfb), though interactive inspection remains its primary use.

## Common pitfalls

**1. "Fixed Frame does not exist" / blank scene**
This is the single most common issue. RViz2 requires at least one TF broadcaster publishing transforms into the fixed frame before it can render anything. If you're developing a node in isolation, launch a `robot_state_publisher` with your URDF, or at minimum publish a static transform (`ros2 run tf2_ros static_transform_publisher`) from your sensor frame to the fixed frame. Always run `ros2 run tf2_tools view_frames` to verify the TF tree is connected before blaming RViz2.

**2. Timestamp mismatch with simulation / bags**
When replaying a bag or running Gazebo, set `use_sim_time: true` on the RViz2 node (via `parameters` in the launch file, as shown above) and ensure your `/clock` topic is publishing. Without this, TF lookups fail with "Lookup would require extrapolation into the future" because RViz2 uses wall-clock time to query transforms.

**3. Storing `.rviz` files in non-reproducible paths**
Hardcoding an absolute path like `/home/user/my.rviz` in a launch file breaks on every other machine. Always resolve the config path with `get_package_share_directory` or `PathJoinSubstitution(FindPackageShare(...))` and commit the `.rviz` file into your package under `rviz/`. Add the `rviz/` directory to `CMakeLists.txt` / `setup.py` `data_files` so it installs properly.

## Further reading

- [RViz2 package documentation — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/p/rviz2/)
- [RViz User Guide — Vulcanexus / Humble](https://docs.vulcanexus.org/en/humble/ros2_documentation/source/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
- [nav2_bringup `rviz_launch.py` — Navigation2 GitHub](https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/launch/rviz_launch.py)

---

*2026-10-21 | ROS 2: Jazzy / Humble*
