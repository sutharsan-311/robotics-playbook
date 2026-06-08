# ROS2 URDF — Robot Description Format

URDF (Unified Robot Description Format) is the XML schema ROS uses to model a robot's kinematic structure, geometry, and physical properties — it is the single source of truth that feeds `robot_state_publisher`, Gazebo, MoveIt 2, Nav2, and RViz2.

## What it is

A URDF file describes a robot as a tree of **links** (rigid bodies) connected by **joints** (degrees of freedom). Every node in the tree is a coordinate frame; `robot_state_publisher` reads the file and joint states to broadcast transforms on `/tf` and `/tf_static`.

Core joint types you will use in practice:

| Type | Behaviour |
|------|-----------|
| `fixed` | No motion; welds two links |
| `revolute` | Rotation with hard limits |
| `continuous` | Unbounded rotation (wheels) |
| `prismatic` | Linear translation with limits |
| `floating` | 6-DOF (rarely hand-authored) |

Each `<link>` has up to three sub-elements:

- `<visual>` — rendered geometry and material
- `<collision>` — simplified shape used by physics and planners
- `<inertial>` — mass and 3×3 inertia tensor (required for simulation)

## How it works

The snippet below is taken directly from the official `ros/urdf_tutorial` ros2 branch. It shows a fixed leg attached to a base body and a continuous-rotation wheel:

```xml
<!-- Source: github.com/ros/urdf_tutorial/tree/ros2 — urdf/06-flexible.urdf -->
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue"/>
  </visual>
</link>

<link name="right_leg">
  <visual>
    <geometry>
      <box size="0.6 0.1 0.2"/>
    </geometry>
    <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
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
    <origin rpy="1.57075 0 0" xyz="0 0 0"/>
    <geometry>
      <cylinder length="0.1" radius="0.035"/>
    </geometry>
    <material name="black"/>
  </visual>
</link>

<joint name="right_front_wheel_joint" type="continuous">
  <axis xyz="0 1 0"/>
  <parent link="right_base"/>
  <child link="right_front_wheel"/>
  <origin xyz="0.133333333333 0 -0.085"/>
</joint>
```

In real packages, URDF is almost always generated from **Xacro** (XML macro language) to avoid repetition. The `08-macroed.urdf.xacro` in the same repo shows how a reusable `<xacro:macro name="wheel">` collapses four identical wheel definitions into one. Convert at build time:

```bash
xacro my_robot.urdf.xacro > my_robot.urdf
```

Or let `robot_state_publisher` consume xacro directly via the `xacro` Python API in your launch file (the standard pattern since ROS2 Humble).

## Common pitfalls

**1. Zero or placeholder inertia in simulated links**
Setting `ixx`, `iyy`, `izz` to `0.0` or leaving `<inertial>` out causes Gazebo to make the link infinitely light — it becomes numerically unstable and the robot explodes on contact. Use `1e-3` as a safe placeholder for lightweight links, or compute proper values with `xacro` math expressions or tools like `calc-inertia`. The inertia tensor must be positive definite and satisfy the triangle inequalities (`ixx + iyy >= izz`, etc.).

**2. Collision geometry copied verbatim from visual**
High-resolution mesh files make accurate visuals but are catastrophic as collision geometry — physics solvers iterate over every triangle every timestep. Always define a separate `<collision>` element using a simplified primitive (box, cylinder, sphere) or a decimated convex hull.

**3. `package://` paths that work in one workspace but break in another**
Mesh paths like `<mesh filename="package://my_robot_description/meshes/base.stl"/>` resolve through `ament` at runtime. If the package is not sourced or the `install/` directory is missing, `robot_state_publisher` silently loads the URDF but RViz2 renders nothing. Always verify with `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro path/to/robot.urdf.xacro)"` before integrating into a launch file.

## Further reading

- [ROS 2 URDF Tutorials (Humble)](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html) — official step-by-step series covering visual models through Xacro macros
- [ros/urdf_tutorial — ros2 branch](https://github.com/ros/urdf_tutorial/tree/ros2) — the canonical working examples referenced in this article
- [robot_state_publisher — Jazzy API docs](https://docs.ros.org/en/jazzy/p/robot_state_publisher/) — node parameters, topic interface, and usage notes

---

*2026-06-08 | ROS2 version: Jazzy / Humble*
