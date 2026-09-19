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

Taken from [`ros/urdf_tutorial` ros2 branch](https://github.com/ros/urdf_tutorial/tree/ros2) (Apache 2.0). It shows a fixed leg and a continuous-rotation wheel:

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

## Inertia tensors — formulas and xacro macros

Every simulated link needs a correct `<inertial>` element. Using incorrect values (or zero) causes Gazebo to produce unstable physics or treat the link as massless.

### Standard formulas

For uniform-density solid primitives (all values in SI units: kg, m):

**Solid cylinder** (radius `r`, length `h`, mass `m`):
```
Ixx = Iyy = (1/12) * m * (3r² + h²)
Izz = (1/2) * m * r²
```

**Solid box** (side lengths `x`, `y`, `z`, mass `m`):
```
Ixx = (1/12) * m * (y² + z²)
Iyy = (1/12) * m * (x² + z²)
Izz = (1/12) * m * (x² + y²)
```

**Solid sphere** (radius `r`, mass `m`):
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

*(Source: [List of moments of inertia — Wikipedia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia); standard reference cited throughout the ROS 2 URDF tutorials)*

### Xacro inertia macros

Put these in a shared `inertia_macros.xacro` and include it from every robot description file. Xacro evaluates `${}` expressions with Python math, including the `**` power operator:

```xml
<!-- Source pattern: ros-planning/navigation2_tutorials — sam_bot_description.urdf (Apache 2.0) -->

<!-- ─── cylinder ────────────────────────────────────────────────── -->
<!-- m: mass (kg), r: radius (m), h: height/length (m)             -->
<!-- Symmetry axis assumed to be z.                                 -->
<xacro:macro name="cylinder_inertia" params="m r h">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(1/12) * m * (3 * r**2 + h**2)}"  ixy="0.0"  ixz="0.0"
             iyy="${(1/12) * m * (3 * r**2 + h**2)}"  iyz="0.0"
             izz="${(1/2)  * m * r**2}"/>
  </inertial>
</xacro:macro>

<!-- ─── box ───────────────────────────────────────────────────── -->
<!-- m: mass (kg), x/y/z: full side lengths (m)                   -->
<xacro:macro name="box_inertia" params="m x y z">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(1/12) * m * (y**2 + z**2)}"  ixy="0.0"  ixz="0.0"
             iyy="${(1/12) * m * (x**2 + z**2)}"  iyz="0.0"
             izz="${(1/12) * m * (x**2 + y**2)}"/>
  </inertial>
</xacro:macro>

<!-- ─── sphere ────────────────────────────────────────────────── -->
<!-- m: mass (kg), r: radius (m)                                   -->
<xacro:macro name="sphere_inertia" params="m r">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(2/5) * m * r**2}"  ixy="0.0"  ixz="0.0"
             iyy="${(2/5) * m * r**2}"  iyz="0.0"
             izz="${(2/5) * m * r**2}"/>
  </inertial>
</xacro:macro>
```

Usage — one-line inertial block per link:

```xml
<link name="base_link">
  <xacro:cylinder_inertia m="2.0" r="0.2" h="0.6"/>
  <visual> ... </visual>
  <collision> ... </collision>
</link>

<link name="left_wheel">
  <xacro:cylinder_inertia m="0.5" r="0.05" h="0.04"/>
  <visual> ... </visual>
  <collision> ... </collision>
</link>
```

> **Off-diagonal terms:** The `ixy`, `ixz`, `iyz` cross-products are zero for any link whose geometry is symmetric about all three axes through the centre of mass — which is true for all three primitives above. For asymmetric CAD-imported meshes, compute the full tensor with a tool like Meshlab (Filters → Quality Measure → Compute Geometric Measures) or the `calc-inertia` ROS package.

*(Source: [Adding physical and collision properties — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html))*

---

## Xacro — macros for real robots

Hand-writing a full URDF for a robot with multiple links means copy-pasting identical geometry blocks. **Xacro** (XML macro language) eliminates this. Every production robot description is a `.urdf.xacro` file, not a plain `.urdf`.

### Properties (constants)

```xml
<!-- Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html -->
<xacro:property name="width"     value="0.2"/>
<xacro:property name="leglen"    value="0.6"/>
<xacro:property name="bodylen"   value="0.6"/>
<xacro:property name="wheeldiam" value="0.07"/>
<xacro:property name="pi"        value="${math.pi}"/>
```

Using `${math.pi}` is the idiomatic form in Jazzy xacro — it resolves to the Python `math.pi` constant rather than a hardcoded approximation.

### Including external xacro files

Split large descriptions into purpose-specific files and include them:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

  <!-- Shared colour / material definitions -->
  <xacro:include filename="$(find-pkg-share my_robot_description)/urdf/materials.xacro"/>

  <!-- Reusable inertia macros -->
  <xacro:include filename="$(find-pkg-share my_robot_description)/urdf/inertia_macros.xacro"/>

  <!-- ros2_control hardware tags — separate file keeps the main URDF readable -->
  <xacro:include filename="$(find-pkg-share my_robot_description)/urdf/my_robot.ros2_control.xacro"/>

</robot>
```

`$(find-pkg-share <pkg>)` is a xacro-only substitution that resolves to the package's `share/` directory in the install tree — the same directory that `get_package_share_directory()` returns in Python launch files.

### Conditional blocks — simulation vs real hardware

Use `<xacro:arg>` + `<xacro:if>` / `<xacro:unless>` to produce different URDF from the same source file depending on whether you are running in simulation or on real hardware:

```xml
<!-- Declare a command-line argument with a default value -->
<xacro:arg name="use_sim" default="false"/>

<!-- Gazebo plugin block — only included when use_sim:=true -->
<xacro:if value="$(arg use_sim)">
  <gazebo>
    <plugin filename="libgz_ros2_control-system.so"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find-pkg-share my_robot_bringup)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>
</xacro:if>

<!-- Real hardware plugin — only included when use_sim is false -->
<xacro:unless value="$(arg use_sim)">
  <ros2_control name="RealHardware" type="system">
    <hardware>
      <plugin>my_robot_hardware/MyRobotHardware</plugin>
      <param name="port">/dev/ttyUSB0</param>
    </hardware>
  </ros2_control>
</xacro:unless>
```

Pass the argument via `xacro` CLI or from a launch file:

```bash
# CLI
xacro my_robot.urdf.xacro use_sim:=true -o /tmp/my_robot_sim.urdf

# Launch file — forward a LaunchArgument into xacro
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': Command([
            'xacro ', LaunchConfiguration('model'),
            ' use_sim:=', LaunchConfiguration('use_sim'),
        ])
    }],
)
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
    <xacro:cylinder_inertia m="0.1" r="${wheeldiam/2}" h="0.1"/>
  </link>
  <joint name="${prefix}_${suffix}_wheel_joint" type="continuous">
    <axis xyz="0 1 0"/>
    <parent link="${prefix}_base"/>
    <child link="${prefix}_${suffix}_wheel"/>
    <origin xyz="${reflect * baselen/3} 0 -${wheeldiam/2 + 0.05}"/>
  </joint>
</xacro:macro>

<!-- Four wheels, four lines -->
<xacro:wheel prefix="right" suffix="front" reflect="1"/>
<xacro:wheel prefix="right" suffix="back"  reflect="-1"/>
<xacro:wheel prefix="left"  suffix="front" reflect="1"/>
<xacro:wheel prefix="left"  suffix="back"  reflect="-1"/>
```

### Mesh geometry — loading STL/DAE files

For a robot built from CAD, the `<visual>` and `<collision>` elements reference mesh files via the `package://` URI scheme:

```xml
<link name="base_link">
  <visual>
    <!-- package://  resolves through ament at runtime, not at file-parse time -->
    <geometry>
      <mesh filename="package://my_robot_description/meshes/base_link.stl"
            scale="0.001 0.001 0.001"/>  <!-- scale: mm → m if CAD exported in mm -->
    </geometry>
    <material name="grey"/>
  </visual>
  <collision>
    <!-- Use a primitive or decimated hull — never the high-poly visual mesh -->
    <geometry>
      <box size="0.4 0.3 0.1"/>
    </geometry>
  </collision>
  <xacro:box_inertia m="3.0" x="0.4" y="0.3" z="0.1"/>
</link>
```

**Supported formats:** STL (most universal, no colour), DAE/Collada (colour and texture, needed for RViz2 colouring). GLTF support was added in Jazzy via `rviz_default_plugins`; Gazebo Harmonic also supports GLTF natively.

Install mesh files to the share directory in `CMakeLists.txt`:
```cmake
install(DIRECTORY meshes urdf launch config
  DESTINATION share/${PROJECT_NAME}
)
```

### Converting at the command line

```bash
# One-shot conversion to inspect the generated XML
xacro my_robot.urdf.xacro -o /tmp/my_robot.urdf

# Pass arguments (for parametric models)
xacro my_robot.urdf.xacro use_sim:=true prefix:=left > /tmp/my_robot_sim.urdf
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
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Set to true when launching with Gazebo',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', LaunchConfiguration('model'),
                    ' use_sim:=', LaunchConfiguration('use_sim'),
                ])
            }],
        ),
        # joint_state_publisher_gui publishes fake joint states for RViz2 visualisation
        # Remove this when real hardware or a Gazebo sim provides /joint_states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
    ])
```

`Command(['xacro ', ...])` is evaluated at launch time — it runs `xacro <path>` as a subprocess and injects the stdout string as the `robot_description` parameter. This is the approach used by `nav2_bringup`, `moveit_configs_utils`, and virtually every production launch stack.

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

If your robot uses `ros2_control` (hardware interfaces, controllers), the URDF is also where you declare the hardware plugin and joint interfaces.

> **Gazebo Classic vs Gazebo (gz):** ROS 2 Jazzy pairs with **Gazebo Harmonic** (the `gz` family — package prefix `gz_ros2_control`). Gazebo Classic (`gazebo_ros2_control`, `libgazebo_ros2_control.so`) was never released for Jazzy/Noble. If you are on Humble and still using Gazebo Classic, replace `gz_ros2_control/GazeboSimSystem` with `gazebo_ros2_control/GazeboSystem` and the plugin filename with `libgazebo_ros2_control.so`.

The following snippet wires a differential-drive robot to `gz_ros2_control` (Jazzy / Gazebo Harmonic):

```xml
<!-- Source: control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html -->
<!-- Place inside <robot> after your link/joint definitions -->

<!-- 1. ros2_control hardware tag — parsed by the controller manager -->
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
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

<!-- 2. Gazebo (gz) plugin tag — tells gz-sim to load the controller manager -->
<gazebo>
  <plugin filename="libgz_ros2_control-system.so"
          name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find-pkg-share my_robot_bringup)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### controllers.yaml — diff_drive_controller

The `controllers.yaml` referenced by the Gazebo plugin configures which controllers the controller manager loads and their parameters. For a differential-drive base:

```yaml
# Source: control.ros.org/jazzy/doc/ros2_control_demos/example_2/doc/userdoc.html
# (DiffBot example — ros-controls/ros2_control_demos, Apache 2.0)
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz — must be >= controller publish rates

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.3        # metres, centre-to-centre
    wheel_radius:     0.05       # metres

    # Calibration multipliers — adjust if odometry drifts systematically
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0           # Hz — /odom publish frequency
    odom_frame_id: odom
    base_frame_id: base_link

    pose_covariance_diagonal:  [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    open_loop: false             # true = integrate cmd_vel instead of encoder feedback
    enable_odom_tf: true         # publish odom → base_link transform on /tf

    # Stop publishing if no cmd_vel received for this many seconds
    cmd_vel_timeout: 0.5
    use_stamped_vel: false       # set true if sending TwistStamped instead of Twist
```

The `joint_state_broadcaster` must always be loaded first — it publishes `/joint_states` that `robot_state_publisher` uses to update the TF tree. Load controllers from the launch file or via the CLI:

```bash
# Activate broadcasters and controllers after the controller manager starts
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_controller

# Verify
ros2 control list_controllers
```

**`package.xml`** dependencies for simulation:
```xml
<exec_depend>gz_ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
<exec_depend>diff_drive_controller</exec_depend>
```

Install packages on Jazzy:
```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```

---

## Common pitfalls

**1. Zero or placeholder inertia in simulated links.**
Leaving `<inertial>` out, or setting all diagonal values to `0.0`, causes Gazebo to treat the link as massless — contacts trigger infinite acceleration and the robot immediately explodes in the simulation. Use the xacro inertia macros above with computed values. If exact mass is unknown, use `1e-3` as a placeholder (it implies a ~1 g link with a tiny moment of inertia) but replace it before tuning dynamics.

The inertia tensor must be **positive semi-definite** and obey the triangle inequalities:
`ixx + iyy ≥ izz`, `ixx + izz ≥ iyy`, `iyy + izz ≥ ixx`.
The xacro macros above satisfy this automatically for uniform-density primitives.

**2. Collision geometry copied verbatim from visual.**
High-resolution mesh files in `<collision>` force the physics solver to iterate over every triangle every timestep — a 50k-polygon mesh as collision geometry will drop a Gazebo simulation to under 1 Hz. Always define a separate `<collision>` element with a primitive (box, cylinder, sphere) or a decimated convex hull (generated offline with VHACD or Blender Decimate).

**3. `package://` paths that break in other workspaces.**
Paths like `<mesh filename="package://my_robot_description/meshes/base.stl"/>` resolve through `ament` at runtime. If the package is not sourced or the `install/` directory is missing, `robot_state_publisher` loads the URDF silently but RViz2 renders nothing and logs a warning about unreachable mesh URIs. Validate end-to-end with:
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro path/to/robot.urdf.xacro)"
```

**4. Editing the generated `.urdf` file instead of the `.urdf.xacro`.**
The `Command(['xacro ', ...])` pattern regenerates the URDF from xacro at every launch. Any hand-edits to a generated `.urdf` file are silently overwritten at the next launch. The xacro source is the only file to edit.

**5. Publishing the robot description before `robot_state_publisher` starts.**
Any node that reads `robot_description` (e.g. MoveIt 2's move_group, nav2_bringup) may start before `robot_state_publisher` has written the parameter. Use `ros2 param get /robot_state_publisher robot_description` to confirm it is live, or use a `RegisterEventHandler(OnProcessStart(...))` to sequence dependent nodes.

**6. STL files exported in millimetres from CAD.**
Most CAD tools (SolidWorks, Fusion 360, CATIA) export STL in millimetres; URDF works in metres. Add `scale="0.001 0.001 0.001"` to the `<mesh>` tag. Forgetting this makes your robot appear 1000× larger — it will be visible as an enormous mesh in RViz2 and will break every collision check.

**7. Missing `joint_state_broadcaster` in controllers.yaml.**
The `diff_drive_controller` reads wheel joint positions and velocities from the hardware interface, but `robot_state_publisher` needs `/joint_states` for TF. If `joint_state_broadcaster` is not loaded, all `base_link → wheel` transforms are stale or missing — Nav2 will fail immediately with TF errors even though the diff drive controller is active.

---

## Further reading

- [Building a Visual Robot Model — ROS 2 URDF Tutorial Series (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) — step-by-step series from basic links through Gazebo integration
- [Adding Physical and Collision Properties — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html) — mass, inertia, and collision geometry; sources the Wikipedia moment-of-inertia table
- [Using Xacro to Clean Up a URDF File — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) — properties, math expressions, macros, conditionals, and `<xacro:include>`
- [Using URDF with robot_state_publisher (Python) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.html) — canonical launch file pattern with Command substitution
- [ros/urdf_tutorial — ros2 branch (GitHub)](https://github.com/ros/urdf_tutorial/tree/ros2) — 09 worked URDF/xacro examples referenced throughout the official docs
- [navigation2_tutorials — sam_bot_description (GitHub)](https://github.com/ros-planning/navigation2_tutorials/blob/master/sam_bot_description/src/description/sam_bot_description.urdf) — full diff-drive URDF with sphere/cylinder/box inertia macros and ros2_control tags
- [robot_state_publisher — Jazzy package docs](https://docs.ros.org/en/jazzy/p/robot_state_publisher/) — node parameters, topic interface, and `/robot_description` topic vs parameter behaviour
- [DiffBot example — ros2_control Jazzy docs](https://control.ros.org/jazzy/doc/ros2_control_demos/example_2/doc/userdoc.html) — complete diff_drive_controller URDF tags, controllers.yaml, and launch file pattern
- [diff_drive_controller — ROS2_Control Jazzy docs](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) — all parameters: wheel_separation, wheel_radius, open_loop, enable_odom_tf, cmd_vel_timeout
- [gz_ros2_control — ROS 2 Control Jazzy docs](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) — wiring ros2_control through the URDF into Gazebo Harmonic; GazeboSimSystem plugin, hold_joints

---

*2026-09-19 | ROS2 version: Jazzy (Gazebo Harmonic) / Humble (Gazebo Classic — deprecated)*
