# ROS2 Custom Messages and Interfaces

Custom interfaces let you define domain-specific data structures — beyond what standard `std_msgs` or `geometry_msgs` offer — and share them cleanly across your entire ROS2 system.

## What it is

ROS2 supports three interface types, each serving a different communication pattern:

| Type | File ext | Pattern | Parts |
|---|---|---|---|
| Message | `.msg` | Pub/Sub | Fields only |
| Service | `.srv` | Request/Response | Request `---` Response |
| Action | `.action` | Long-running goal | Goal `---` Result `---` Feedback |

All three are defined as plain text files with typed fields. At build time, `rosidl_default_generators` compiles them into C++ headers and Python modules, making them importable like any first-class package.

**Critical constraint:** interfaces can only be defined inside `ament_cmake` packages. You cannot put them in an `ament_python` package.

## How it works

### 1. Create a dedicated interface package

```bash
ros2 pkg create --build-type ament_cmake my_robot_interfaces
```

Organise files by type:

```
my_robot_interfaces/
├── msg/
│   └── SensorReading.msg
├── srv/
│   └── SetSpeed.srv
├── action/
│   └── NavigateToGoal.action
├── CMakeLists.txt
└── package.xml
```

### 2. Define the interface files

**`msg/SensorReading.msg`** — from [ROS2 official tutorial pattern](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html):
```
std_msgs/Header header
float64 distance
float64 bearing
bool valid
```

**`srv/SetSpeed.srv`:**
```
float32 linear_speed
float32 angular_speed
---
bool success
string message
```

**`action/NavigateToGoal.action`** — same three-section pattern as [`nav2_msgs/action/NavigateToPose`](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action):
```
geometry_msgs/PoseStamped target_pose
---
std_msgs/Empty result
---
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

### 3. Configure `package.xml`

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### 4. Configure `CMakeLists.txt`

```cmake
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "srv/SetSpeed.srv"
  "action/NavigateToGoal.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
```

The `DEPENDENCIES` argument is mandatory whenever any field references a type from another package. Omitting it produces cryptic link errors at colcon build time.

### 5. Build and verify

```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash
ros2 interface show my_robot_interfaces/msg/SensorReading
```

## Common pitfalls

**1. Defining interfaces inside an `ament_python` package**
This silently fails or produces a confusing CMake error. Interfaces require `ament_cmake`. If you want to keep Python nodes alongside the interface definitions, use `ament_cmake` with `ament_cmake_python` — or, more cleanly, keep interfaces in their own dedicated package.

**2. Forgetting `DEPENDENCIES` when composing from existing message types**
If `SensorReading.msg` uses `std_msgs/Header` but `std_msgs` is not listed in `DEPENDENCIES`, the build succeeds but the generated code will not resolve the type at link time. Always declare every upstream interface package in both `package.xml` (as `exec_depend`) and in the `DEPENDENCIES` clause of `rosidl_generate_interfaces`.

**3. Using a custom interface in the same package that defines it**
Downstream packages only need `find_package(my_robot_interfaces REQUIRED)`. But if a C++ node lives *inside* the same package as the interface, you need the additional CMake call [`rosidl_get_typesupport_target()`](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) to link correctly. The common fix is to put interfaces in their own package and avoid this entirely.

## Further reading

- [Creating custom msg and srv files — ROS 2 Humble docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [About ROS 2 Interfaces (concepts) — docs.ros.org](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html)
- [Implementing custom interfaces in a single package — ROS 2 docs](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)

---
*2026-09-16 | ROS2 version: Jazzy / Humble*
