# ROS2 Custom Messages and Interfaces

Custom interfaces let you define domain-specific data structures — beyond what `std_msgs` or `geometry_msgs` offer — and share them cleanly across your entire ROS2 system. This article covers the full workflow: defining interfaces, understanding what gets generated, wiring them up in C++ and Python, and diagnosing problems with the CLI.

---

## Interface types

ROS2 supports three interface types, each serving a different communication pattern:

| Type | File ext | Pattern | Sections |
|---|---|---|---|
| Message | `.msg` | Pub/Sub | Fields only |
| Service | `.srv` | Request/Response | Request `---` Response |
| Action | `.action` | Long-running goal | Goal `---` Result `---` Feedback |

All three are plain text files with typed fields. `rosidl_default_generators` compiles them into C++ headers and Python modules at build time.

**Critical constraint:** interfaces can only be defined inside `ament_cmake` packages. `ament_python` packages cannot host interface definitions.

---

## Field type reference

### Primitive types

```
bool      byte      char
float32   float64
int8      uint8
int16     uint16
int32     uint32
int64     uint64
string    wstring
```

### Arrays

| Specifier | Meaning |
|---|---|
| `float64[3]` | Static array, exactly 3 elements |
| `float64[]` | Unbounded dynamic array |
| `float64[<=10]` | Bounded dynamic array, max 10 elements |

### Bounded strings

```
string<=256 label       # UTF-8 string, max 256 bytes
```

### Nested types

```
std_msgs/Header header         # embed a full message from another package
geometry_msgs/Pose base_pose
MyCustomMsg/SensorReading reading  # same-package nesting (use package name)
```

### Constants

Constants are defined inline in `.msg` or `.srv` files and become static class members in the generated code. They cannot be used as field default values.

```
# In a .msg file — type NAME=VALUE syntax
uint8 STATUS_IDLE=0
uint8 STATUS_RUNNING=1
uint8 STATUS_ERROR=2

uint8 status    # this is a mutable field, not a constant
```

Constants support decimal, binary (`0b01`), octal (`0o01`), and hexadecimal (`0x01`) literals for integer types.

### Field default values

A third token on a field line sets its default:

```
float64 speed 0.0
bool active false
string label "unnamed"
int32[3] counts [0, 0, 0]
```

**Limitation:** default values are not supported for complex types (nested messages) or string arrays.

---

## Package structure

Keep interfaces in a dedicated `ament_cmake` package. Co-locating interface definitions with node code is possible but requires extra CMake steps (see [Same-package usage](#same-package-usage) below) and makes the interface harder to reuse across your stack.

```bash
ros2 pkg create --build-type ament_cmake my_robot_interfaces
```

```
my_robot_interfaces/
├── msg/
│   └── SensorReading.msg
├── srv/
│   └── SetSpeed.srv
├── action/
│   └── NavigateToPose.action
├── CMakeLists.txt
└── package.xml
```

**Naming rules:**
- Filenames must be `UpperCamelCase` (e.g. `SensorReading.msg`) — this determines the generated type name.
- Field names must be `snake_case`.
- Constant names must be `ALL_CAPS_SNAKE_CASE`.

---

## Interface file examples

### `msg/SensorReading.msg`

```
# Header carries timestamp and frame_id — always useful for sensor data
std_msgs/Header header

float64 distance        # metres, positive forward
float64 bearing         # radians, positive CCW from heading
bool valid              # false if sensor reports fault

# Status constants
uint8 STATUS_OK=0
uint8 STATUS_OUT_OF_RANGE=1
uint8 STATUS_FAULT=2
uint8 status            # one of the STATUS_* constants above
```

### `srv/SetSpeed.srv`

```
float32 linear_speed    # m/s
float32 angular_speed   # rad/s
---
bool success
string message
```

### `action/NavigateToPose.action`

```
# Goal — sent once by the client to start the task
geometry_msgs/PoseStamped target_pose
float32 tolerance       # metres, acceptable arrival radius
---
# Result — returned once when the action finishes (success or abort)
bool reached
geometry_msgs/Pose final_pose
---
# Feedback — streamed by the server during execution
geometry_msgs/PoseStamped current_pose
float32 distance_remaining  # metres to goal
float32 eta_seconds
```

*(Pattern mirrors [`nav2_msgs/action/NavigateToPose`](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action))*

---

## Build configuration

### `package.xml`

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

### `CMakeLists.txt`

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "srv/SetSpeed.srv"
  "action/NavigateToPose.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

The `DEPENDENCIES` clause is mandatory for every upstream interface package referenced in any field. Omitting it produces cryptic link errors at build time, not at the `.msg` file level.

### Build and verify

```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash

# Inspect the generated interface definition
ros2 interface show my_robot_interfaces/msg/SensorReading
ros2 interface show my_robot_interfaces/srv/SetSpeed
ros2 interface show my_robot_interfaces/action/NavigateToPose

# List everything your package exports
ros2 interface list | grep my_robot_interfaces

# Print a minimal instantiation template (useful for scripting)
ros2 interface proto my_robot_interfaces/msg/SensorReading
```

---

## Generated code anatomy

Understanding what `rosidl` generates helps you include the right headers and import the right symbols.

### C++ — header path convention

`rosidl` converts `UpperCamelCase` filenames to `snake_case` headers:

| Interface file | C++ header |
|---|---|
| `msg/SensorReading.msg` | `my_robot_interfaces/msg/sensor_reading.hpp` |
| `srv/SetSpeed.srv` | `my_robot_interfaces/srv/set_speed.hpp` |
| `action/NavigateToPose.action` | `my_robot_interfaces/action/navigate_to_pose.hpp` |

The C++ namespace mirrors the package and subdirectory:

```cpp
my_robot_interfaces::msg::SensorReading
my_robot_interfaces::srv::SetSpeed
my_robot_interfaces::action::NavigateToPose
```

### Python — module path convention

```python
from my_robot_interfaces.msg import SensorReading
from my_robot_interfaces.srv import SetSpeed
from my_robot_interfaces.action import NavigateToPose
```

### Action generated sub-types

A single `.action` file generates six sub-types used internally by `rclcpp_action` / `rclpy.action`:

```cpp
NavigateToPose::Goal
NavigateToPose::Result
NavigateToPose::Feedback
NavigateToPose::Impl::SendGoalService      // wraps goal + UUID
NavigateToPose::Impl::GetResultService     // wraps result + status
NavigateToPose::Impl::FeedbackMessage      // wraps feedback + UUID
```

You only use `Goal`, `Result`, and `Feedback` directly. The `Impl::*` types are consumed by the action middleware.

---

## Using interfaces from another package (C++)

### `package.xml` for the consuming package

```xml
<depend>my_robot_interfaces</depend>
```

### `CMakeLists.txt` for the consuming package

```cmake
find_package(my_robot_interfaces REQUIRED)

add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node rclcpp my_robot_interfaces)
install(TARGETS sensor_node DESTINATION lib/${PROJECT_NAME})
```

### Publisher

```cpp
// Source pattern: docs.ros.org/en/jazzy/Tutorials (Apache 2.0)
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/sensor_reading.hpp"

class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher() : Node("sensor_publisher")
  {
    pub_ = create_publisher<my_robot_interfaces::msg::SensorReading>(
      "sensor_data", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        auto msg = my_robot_interfaces::msg::SensorReading();
        msg.header.stamp = now();
        msg.header.frame_id = "lidar_link";
        msg.distance = 2.34;
        msg.bearing = 0.0;
        msg.valid = true;
        msg.status = my_robot_interfaces::msg::SensorReading::STATUS_OK;
        pub_->publish(msg);
      });
  }
private:
  rclcpp::Publisher<my_robot_interfaces::msg::SensorReading>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

### Subscriber

```cpp
#include "my_robot_interfaces/msg/sensor_reading.hpp"

void callback(const my_robot_interfaces::msg::SensorReading & msg)
{
  RCLCPP_INFO(rclcpp::get_logger("sub"), "dist=%.2f valid=%d", msg.distance, msg.valid);
}

// In node constructor:
sub_ = create_subscription<my_robot_interfaces::msg::SensorReading>(
  "sensor_data", 10, callback);
```

---

## Using interfaces from another package (Python)

```python
# Source pattern: docs.ros.org/en/jazzy/Tutorials (Apache 2.0)
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import SensorReading

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.pub = self.create_publisher(SensorReading, 'sensor_data', 10)
        self.timer = self.create_timer(0.1, self.publish_cb)

    def publish_cb(self):
        msg = SensorReading()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.distance = 2.34
        msg.valid = True
        msg.status = SensorReading.STATUS_OK  # constant access
        self.pub.publish(msg)
```

Constants are accessible directly on the class: `SensorReading.STATUS_OK`.

---

## Same-package usage

When a C++ node lives in the **same package** as the interface definition, `ament_target_dependencies` is not enough — the generated library is not yet known to CMake at dependency resolution time. Use `rosidl_get_typesupport_target` instead:

```cmake
# After rosidl_generate_interfaces(...)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node "${cpp_typesupport_target}")
ament_target_dependencies(my_node rclcpp)
```

`rosidl_target_interfaces` (the older call) is deprecated as of Humble and removed in Rolling. Always use `rosidl_get_typesupport_target`.

*(Source: [ros2/rosidl — rosidl_get_typesupport_target.cmake, jazzy](https://github.com/ros2/rosidl/blob/jazzy/rosidl_cmake/cmake/rosidl_get_typesupport_target.cmake))*

Python nodes in the same package do not need this extra step — the ament Python path resolution handles it automatically.

---

## Diagnostics

```bash
# Show the full field and constant definition after build
ros2 interface show my_robot_interfaces/msg/SensorReading

# List all interfaces exported by your package
ros2 interface list | grep my_robot_interfaces

# Print a zero-filled YAML template (useful for ros2 topic pub scripting)
ros2 interface proto my_robot_interfaces/msg/SensorReading

# Verify topic is being published with the right type
ros2 topic info /sensor_data -v

# Inspect a live message on the wire
ros2 topic echo /sensor_data

# Confirm the service interface after build
ros2 interface show my_robot_interfaces/srv/SetSpeed

# Confirm the action interface and its six sub-types
ros2 interface show my_robot_interfaces/action/NavigateToPose
```

---

## Common pitfalls

**1. Defining interfaces inside an `ament_python` package.**  
`rosidl_default_generators` requires `ament_cmake`. The build fails with a confusing CMake error about `rosidl_generate_interfaces` not being found. Keep interfaces in their own `ament_cmake` package, or use `ament_cmake_python` if you must co-locate Python nodes.

**2. Missing `DEPENDENCIES` in `rosidl_generate_interfaces`.**  
If `SensorReading.msg` uses `std_msgs/Header` but `std_msgs` is not listed in `DEPENDENCIES`, the build succeeds but the generated code will not resolve the type at link time. Declare every upstream interface package in both `package.xml` (`exec_depend`) and in the `DEPENDENCIES` clause.

**3. Using `rosidl_target_interfaces` (deprecated) instead of `rosidl_get_typesupport_target`.**  
`rosidl_target_interfaces` was deprecated in Humble and removed in Rolling. Using it in Jazzy produces a CMake warning that becomes a hard error on Rolling. Migrate to `rosidl_get_typesupport_target`.

**4. Filename is `snake_case` instead of `UpperCamelCase`.**  
`rosidl` requires filenames to be `UpperCamelCase` (e.g., `SensorReading.msg`, not `sensor_reading.msg`). A lowercase filename builds silently on some distros but the generated C++ type name will be mangled and not match the convention expected by `#include` paths.

**5. Field name uses `CamelCase` instead of `snake_case`.**  
Field names must be `snake_case`. `CamelCase` field names cause an `idl_parser` error at build time: `Field name '<name>' does not conform to snake_case`.

**6. Same-package C++ node fails to link.**  
`ament_target_dependencies(my_node rclcpp my_robot_interfaces)` does not work when both the interface and the node are in the same package — CMake cannot resolve a target that is being generated in the same build step. Use `rosidl_get_typesupport_target` + `target_link_libraries` as shown above.

**7. Constants are not default values.**  
Constants defined in a `.msg` file (`uint8 STATUS_OK=0`) are immutable class-level values. They cannot be assigned as the default of a mutable field (`uint8 status STATUS_OK` is a syntax error). Set field defaults with a plain literal: `uint8 status 0`.

**8. `ros2 interface show` returns "not found" after build.**  
This almost always means `source install/setup.bash` was not re-run after the build, so the new package is not on `AMENT_PREFIX_PATH`. Source the overlay and retry.

---

## Further reading

- [Creating custom msg and srv files — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) — official step-by-step: `.msg`, `.srv`, build, C++ and Python usage
- [Implementing custom interfaces in a single package — ROS 2 Iron docs](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html) — same-package pattern using `rosidl_get_typesupport_target`
- [About ROS 2 Interfaces — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html) — complete field type reference, array specifiers, constants, bounded strings, default values
- [Interface definition (.msg/.srv/.action) — design.ros2.org](https://design.ros2.org/articles/legacy_interface_definition.html) — authoritative syntax spec: all types, specifiers, constant literals, naming rules
- [Generated Python interfaces — design.ros2.org](https://design.ros2.org/articles/generated_interfaces_python.html) — Python module layout, constant access, field initialisation, array semantics
- [rosidl_get_typesupport_target.cmake — ros2/rosidl, jazzy (GitHub)](https://github.com/ros2/rosidl/blob/jazzy/rosidl_cmake/cmake/rosidl_get_typesupport_target.cmake) — canonical CMake function signature and usage notes
- [nav2_msgs action definitions — ros-navigation/navigation2 (GitHub)](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action) — real-world action interface reference from the Nav2 stack
- [Writing an action server and client (C++) — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) — consuming action types with `rclcpp_action`: `Goal`, `Result`, `Feedback`, send/cancel patterns

---

*2026-10-10 | ROS2 version: Jazzy / Humble*
