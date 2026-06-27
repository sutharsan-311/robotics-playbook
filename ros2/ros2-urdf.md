# ROS2 URDF — Robot Description Format

URDF (Unified Robot Description Format) is the XML schema ROS uses to model a robot's kinematic structure, geometry, and physical properties. It is the single source of truth that feeds `robot_state_publisher`, Gazebo, MoveIt 2, Nav2, and RViz2 — every downstream system derives its model of your robot from this file.

---

## What it is

A URDF file describes a robot as a tree of **links** (rigid bodies) connected by **joints** (degrees of freedom). Every node in the tree is a coordinate frame; `robot_state_publisher` reads the file and live joint states to broadcast transforms on `/tf` and `/tf_static`.

Core joint types:

| Type | Behaviour |
|------|-----------|
| `fixed` | No motion; welds two links |
| `revolute` | Rotation with hard limits (`<limit>`) |
| `continuous` | Unbounded rotation — wheels, rollers |
| `prismatic` | Linear translation with limits |
| `floating` | 6-DOF (rarely hand-authored; prefer fixed + software offset) |

Each `<link>` carries up to three sub-elements:

- `<visual>` — rendered geometry and material
- `<collision>` — simplified shape used by physics and planners
- `<inertial>` — mass and 3×3 inertia tensor (required for simulation; omit it and Gazebo treats the link as massless → immediate numerical explosion)

---

## Basic link–joint structure

The snippet below is taken directly from the [`ros/urdf_tutorial` ros2 branch](https://github.com/ros/urdf_tutorial/tree/ros2) (Apache 2.0). It shows a fixed leg and a continuous-rotation wheel:

```xml
<!-- Source: github.com/ros/urdf_tutorial/blob/ros2/urdf/06-flexible.urdf -->
<link name="base_link">
  <visual>
    <geometry><cylinder length="0.6" radius="0.2"/></geometry>
    <material name="blue"/>
  </visual>
</link>

<link name="right_leg">
  <visual>
    <origin rpy="0 1.5708 0" xyz="0 0 -0.3"/>
    <geometry><box size="0.6 0.1 0.2"/></geometry>
    <material name="white"/>
  </visual>
</link>

<joint name="base_to_right_leg" type="fixed">
  <parent link="base_link"/>
  <child link="right_leg"/>
  <origin xyz="0 -0.22 0.25"/>
</joint>

<link name="right_front_wheel">
  <visual>
    <origin rpy="1.5708 0 0" xyz="0 0 0"/>
    <geometry><cylinder length="0.1" radius="0.035"/></geometry>
    <material name="black"/>
  </visual>
</link>

<joint name="right_front_wheel_joint" type="continuous">
  <axis xyz="0 1 0"/>
  <parent link="right_base"/>
  <child link="right_front_wheel"/>
  <origin xyz="0.133333 0 -0.085"/>
</joint>
```

---

## Xacro — macros for real robots

Hand-writing a full URDF for a robot with four wheels and ten links means copy-pasting the same geometry blocks dozens of times. **Xacro** (XML macro language) eliminates this. In practice, every production robot description is a `.urdf.xacro` file, not a plain `.urdf`.

### Properties (constants)

```xml
<!-- Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html -->
<xacro:property name="width" value="0.2"/>
<xacro:property name="leglen" value="0.6"/>
<xacro:property name="polelen" value="0.2"/>
<xacro:property name="bodylen" value="0.6"/>
<xacro:property name="baselen" value="0.4"/>
<xacro:property name="wheeldiam" value="0.07"/>
<xacro:property name="pi" value="3.1415"/>
```

### Reusable inertia macro

The `default_inertial` macro from `urdf_tutorial/urdf/08-macroed.urdf.xacro` is the standard placeholder pattern for links where exact inertia values are not yet known:

```xml
<!-- Source: github.com/ros/urdf_tutorial/blob/ros2/urdf/08-macroed.urdf.xacro -->
<xacro:macro name="default_inertial" params="mass">
  <inertial>
    <mass value="${mass}"/>
    <inertia ixx="1e-3" ixy="0.0" ixz="0.0"
             iyy="1e-3" iyz="0.0"
             izz="1e-3"/>
  </inertial>
</xacro:macro>
```

Usage anywhere in the same file — one line instead of six:
```xml
<link name="base_link">
  <xacro:default_inertial mass="2.0"/>
  <visual> ... </visual>
  <collision> ... </collision>
</link>
```

### Wheel macro

```xml
<!-- Source: github.com/ros/urdf_tutorial/blob/ros2/urdf/08-macroed.urdf.xacro -->
<xacro:macro name="wheel" params="prefix suffix reflect">
  <link name="${prefix}_${suffix}_wheel">
    <visual>
      <origin rpy="${pi/2} 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder radius="${wheeldiam/2}" length="0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin rpy="${pi/2} 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder radius="${wheeldiam/2}" length="0.1"/>
      </geometry>
    </collision>
    <xacro:default_inertial mass="0.1"/>
  </link>
  <joint name="${prefix}_${suffix}_wheel_joint" type="continuous">
    <axis xyz="0 1 0"/>
    <parent link="${prefix}_base"/>
    <child link="${prefix}_${suffix}_wheel"/>
    <origin xyz="${reflect*baselen/3} 0 -${wheeldiam/2+.05}"/>
  </joint>
</xacro:macro>

<!-- Four wheels, four lines -->
<xacro:wheel prefix="right" suffix="front" reflect="1"/>
<xacro:wheel prefix="right" suffix="back"  reflect="-1"/>
<xacro:wheel prefix="left"  suffix="front" reflect="1"/>
<xacro:wheel prefix="left"  suffix="back"  reflect="-1"/>
```

### Converting at the command line

```bash
# One-shot conversion to inspect the generated XML
xacro my_robot.urdf.xacro -o /tmp/my_robot.urdf

# Pass arguments to the xacro (for parametric models)
xacro my_robot.urdf.xacro use_gripper:=true prefix:=left > /tmp/my_robot.urdf
```

---

## Launch file — robot_state_publisher + xacro

The standard Jazzy pattern feeds xacro output directly to `robot_state_publisher` via the `Command` substitution — no intermediate file on disk, no manual regeneration step.

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.html
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model_path,
            description='Absolute path to the robot URDF/xacro file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('model')])
            }],
        ),
        # joint_state_publisher_gui publishes fake joint states for RViz2 visualisation
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
    ])
```

`Command(['xacro ', LaunchConfiguration('model')])` is evaluated at launch time — it runs `xacro <path>` as a subprocess and injects the stdout string as the `robot_description` parameter. This is the approach used by `nav2_bringup`, `moveit_configs_utils`, and virtually every production launch stack.

**package.xml** must declare:
```xml
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>joint_state_publisher_gui</exec_depend>  <!-- dev/visualisation only -->
```

---

## Validation tooling

```bash
# Install the standalone URDF parser tools (not ROS-version-specific)
sudo apt install liburdfdom-tools

# Check syntax and link tree of a plain URDF
check_urdf /tmp/my_robot.urdf
# Output:
#   robot name is: my_robot
#   ---------- Successfully Parsed XML ---------------
#   root Link: base_link has 3 child(ren)
#       child(1):  left_wheel
#       child(2):  right_wheel
#       child(3):  camera_link

# For xacro files, convert first then check
xacro my_robot.urdf.xacro -o /tmp/my_robot.urdf && check_urdf /tmp/my_robot.urdf

# Render the kinematic tree to a Graphviz PDF
urdf_to_graphviz /tmp/my_robot.urdf
# → generates my_robot.pdf and my_robot.gv in the current directory

# Confirm robot_state_publisher received the description at runtime
ros2 param get /robot_state_publisher robot_description | head -5
```

`check_urdf` catches missing child links, joint chains that don't form a proper tree, and malformed XML. `urdf_to_graphviz` is the fastest way to spot a disconnected subtree or a wrong parent assignment before running a simulation.

---

## ros2_control tags

If your robot uses `ros2_control` (hardware interfaces, controllers), the URDF is also where you declare the hardware plugin and joint interfaces. The following snippet wires a simulated system to `gazebo_ros2_control`:

```xml
<!-- Source: control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>

<!-- Gazebo plugin that parses the ros2_control tags above -->
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot_bringup)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

`<robot_param>` defaults to `robot_description` on the `robot_state_publisher` node — no extra configuration needed if you follow the standard launch pattern.

---

## Common pitfalls

**1. Zero or placeholder inertia in simulated links.**
Leaving `<inertial>` out, or setting all diagonal values to `0.0`, causes Gazebo to treat the link as massless — contacts trigger infinite acceleration and the robot immediately "explodes." Use `1e-3` as a safe placeholder (the `default_inertial` xacro macro above) and replace it with computed values before tuning dynamics.

The inertia tensor must be positive semi-definite and obey the triangle inequalities:
`ixx + iyy ≥ izz`, `ixx + izz ≥ iyy`, `iyy + izz ≥ ixx`.

**2. Collision geometry copied verbatim from visual.**
High-resolution mesh files in `<collision>` force the physics solver to iterate over every triangle every timestep — a 50k-polygon mesh as collision geometry will drop a Gazebo simulation to under 1 Hz. Always define a separate `<collision>` element with a primitive (box, cylinder, sphere) or a decimated convex hull (generated offline with VHACD or Blender).

**3. `package://` paths that break in other workspaces.**
Paths like `<mesh filename="package://my_robot_description/meshes/base.stl"/>` resolve through `ament` at runtime. If the package is not sourced or the `install/` directory is missing, `robot_state_publisher` loads the URDF silently but RViz2 renders nothing and logs a warning about unreachable mesh URIs. Always validate with:
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro path/to/robot.urdf.xacro)"
```
before integrating into a launch file.

**4. Publishing the robot description before `robot_state_publisher` starts.**
Any node that calls `robot_state_publisher` to get the URDF (e.g. MoveIt 2's move_group) may start before `robot_state_publisher` has published the `robot_description` parameter. Use `ros2 param get /robot_state_publisher robot_description` to confirm the parameter is live, or add a `RegisterEventHandler(OnProcessStart(...))` in your launch file to sequence dependent nodes.

**5. Editing the generated `.urdf` file instead of the `.urdf.xacro`.**
The `Command(['xacro ', ...])` pattern regenerates the URDF from xacro at every launch. Any hand-edits to a generated `.urdf` file are silently overwritten. The xacro source is the only file to edit.

---

## Further reading

- [Building a Visual Robot Model — ROS 2 URDF Tutorial Series (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) — step-by-step series from basic links through Gazebo integration
- [Using Xacro to Clean Up a URDF File — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) — properties, math expressions, macros, and conditionals with full worked examples
- [Using URDF with robot_state_publisher (Python) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.html) — the canonical launch file pattern with Command substitution
- [ros/urdf_tutorial — ros2 branch (GitHub)](https://github.com/ros/urdf_tutorial/tree/ros2) — 09 worked URDF/xacro examples referenced throughout the official docs
- [robot_state_publisher — Jazzy package docs](https://docs.ros.org/en/jazzy/p/robot_state_publisher/) — node parameters, topic interface, and `/robot_description` topic vs parameter behaviour
- [gazebo_ros2_control — ROS 2 Control Humble docs](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html) — wiring ros2_control hardware interfaces through the URDF and into Gazebo Classic

---

*2026-06-27 | ROS2 version: Jazzy / Humble*
