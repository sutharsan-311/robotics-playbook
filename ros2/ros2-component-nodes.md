# ROS2 Component Nodes — Composable Executors

Composable nodes let you collapse multiple ROS2 nodes into a single OS process, eliminating DDS serialisation overhead on hot paths and enabling optional zero-copy intra-process communication — without changing any public interfaces.

## What it is

In vanilla ROS2 every node lives in its own executable. Inter-node messages go through the DDS middleware even when both nodes run on the same machine, incurring serialise → transport → deserialise costs for every message.

**Component nodes** (via `rclcpp_components`) break this coupling. A component is built as a **shared library** rather than an executable. At runtime a host process called a **component container** `dlopen`s one or more component libraries and runs them inside a single executor. From the network perspective the nodes still appear separately (their names, topics, parameters, and services are fully visible); only the message path changes.

Three container executables ship with `rclcpp_components`:

| Executable | Executor | Use case |
|---|---|---|
| `component_container` | `SingleThreadedExecutor` | Default; lowest overhead |
| `component_container_mt` | `MultiThreadedExecutor` (shared) | Parallel callbacks across components |
| `component_container_isolated` | One `SingleThreadedExecutor` per component | Timing isolation; each component gets its own thread |

Components can be loaded **statically** (via a launch file) or **dynamically at runtime** using the `ros2 component load` CLI or the `/ComponentManager/load_node` service.

## How it works

### Writing the component (C++)

A component is any class that:
1. Accepts `const rclcpp::NodeOptions &` in its constructor (not `int argc, char** argv`)
2. Registers itself with `RCLCPP_COMPONENTS_REGISTER_NODE`

From the official [`ros2/demos`](https://github.com/ros2/demos/blob/jazzy/composition/src/talker_component.cpp) repository (jazzy branch):

```cpp
#include "composition/talker_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

Talker::Talker(const rclcpp::NodeOptions & options)
: Node("talker", options), count_(0)
{
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  timer_ = create_wall_timer(1s, [this]() { return this->on_timer(); });
}

void Talker::on_timer()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello World: " + std::to_string(++count_);
  pub_->publish(std::move(msg));
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Talker)
```

### Registering in CMakeLists.txt

Build the component as a **shared library**, not an executable, then register it:

```cmake
add_library(talker_component SHARED src/talker_component.cpp)
ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(talker_component "composition::Talker")
```

`rclcpp_components_register_nodes` writes the plugin metadata into the ament index so the container can discover the library by plugin name.  
Use `rclcpp_components_register_node` (singular) if you also want a standalone executable for the same target.

### Launching composably

From the official [`composition_demo_launch.py`](https://github.com/ros2/demos/blob/jazzy/composition/launch/composition_demo_launch.py):

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',          # or _mt / _isolated
    composable_node_descriptions=[
        ComposableNode(package='composition', plugin='composition::Talker', name='talker'),
        ComposableNode(package='composition', plugin='composition::Listener', name='listener'),
    ],
    output='screen',
)
```

### Loading at runtime (no relaunch)

```bash
# Start a bare container
ros2 run rclcpp_components component_container --ros-args -r __node:=ComponentManager

# Load a component into it
ros2 component load /ComponentManager composition composition::Talker

# Inspect what's running
ros2 component list /ComponentManager

# Unload by numeric ID
ros2 component unload /ComponentManager 1
```

### Enabling intra-process communication (IPC)

Add `extra_arguments` to the `ComposableNode` in your launch file:

```python
ComposableNode(
    package='composition',
    plugin='composition::Talker',
    name='talker',
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

With IPC enabled, publishers and subscribers in the **same container** exchange `unique_ptr` messages via a lock-free ring buffer — zero copies, zero DDS round-trips.

## Common pitfalls

1. **Callback groups created after the executor starts don't register.** If you dynamically add a callback group inside a component after the container has already started spinning, its callbacks will be silently ignored. Create all callback groups in the constructor. ([rclcpp issue #2067](https://github.com/ros2/rclcpp/issues/2067))

2. **IPC requires `unique_ptr` publish and matching QoS.** If any subscriber on the same topic uses `std::make_shared` or has a different QoS depth, the IPC path falls back to DDS silently. Always `publish(std::move(msg))` with `std::make_unique` and ensure both sides use compatible QoS to keep the zero-copy path active.

3. **`component_container_isolated` can starve under high load.** Each component gets its own dedicated thread via a separate `SingleThreadedExecutor`. If you load many high-frequency components (camera pipelines, lidars), you can hit CPU saturation before you would with a shared `MultiThreadedExecutor`. Profile with `ros2 run tracetools_analysis` or a perf tool before choosing isolation over sharing. ([rclcpp issue #2295](https://github.com/ros2/rclcpp/issues/2295))

## Further reading

- [Composing multiple nodes in a single process — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html)
- [About Composition — ROS 2 Humble concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html)
- [ros2/demos composition package (jazzy) — GitHub](https://github.com/ros2/demos/tree/jazzy/composition)

---
*2026-07-25 | ROS2 version: Jazzy / Humble*
