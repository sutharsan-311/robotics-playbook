# ROS2 Component Nodes — Composable Executors

Composable nodes let you collapse multiple ROS2 nodes into a single OS process, eliminating DDS serialisation overhead on hot paths and enabling optional zero-copy intra-process communication — without changing any public interfaces or topics.

---

## What it is

In vanilla ROS2 every node lives in its own executable. Inter-node messages go through the DDS middleware even when both nodes run on the same machine, incurring serialise → transport → deserialise costs for every message.

**Component nodes** (via `rclcpp_components`) break this coupling. A component is built as a **shared library** rather than an executable. At runtime a host process called a **component container** `dlopen`s one or more component libraries and runs them inside a single executor. From the network perspective the nodes still appear separately — their names, topics, parameters, and services are fully visible on the ROS graph — only the message delivery path changes.

Three container executables ship with `rclcpp_components`:

| Executable | Executor | Use case |
|---|---|---|
| `component_container` | `SingleThreadedExecutor` | Default; lowest overhead; no blocking callbacks |
| `component_container_mt` | `MultiThreadedExecutor` (shared) | Parallel callbacks across components |
| `component_container_isolated` | One `SingleThreadedExecutor` per component | Fault isolation; a blocking callback in one component cannot stall others |

`component_container_isolated` can optionally give each component a `MultiThreadedExecutor` instead of the default `SingleThreadedExecutor`:
```bash
ros2 run rclcpp_components component_container_isolated \
    --use_multi_threaded_executor
```
*(Source: [rclcpp_components/src/component_container_isolated.cpp — ros2/rclcpp, jazzy](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_components/src/component_container_isolated.cpp))*

Components can be loaded **statically** (via a launch file at startup) or **dynamically at runtime** using the `ros2 component load` CLI or the `/ComponentManager/load_node` service — without stopping the container.

> **Python note:** `rclcpp_components` is a C++-only feature. Python nodes cannot be registered as composable components. Python pipelines that need co-location should use `MultiThreadedExecutor` with multiple nodes added to the same executor instance instead.

---

## Build setup

### CMakeLists.txt

Build the component as a **shared library**, not an executable, then register it with `rclcpp_components_register_nodes`:

```cmake
# Source: github.com/ros2/demos/blob/jazzy/composition/CMakeLists.txt (Apache 2.0)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Build as shared library — NOT add_executable
add_library(talker_component SHARED src/talker_component.cpp)
ament_target_dependencies(talker_component rclcpp rclcpp_components std_msgs)

# Register the plugin name — writes metadata into the ament index
rclcpp_components_register_nodes(talker_component "composition::Talker")

# Install the library so it can be found at runtime
install(TARGETS talker_component DESTINATION lib)
```

`rclcpp_components_register_nodes` (plural) registers all plugins in a library for runtime discovery. Use `rclcpp_components_register_node` (singular) if you also want a standalone executable built from the same target.

### package.xml

```xml
<depend>rclcpp</depend>
<depend>rclcpp_components</depend>
<depend>std_msgs</depend>

<!-- Runtime: the container executables that host your component -->
<exec_depend>rclcpp_components</exec_depend>
```

---

## Writing a component (C++)

A component is any class that:
1. Accepts `const rclcpp::NodeOptions &` in its constructor (not `int argc, char** argv`)
2. Registers itself with `RCLCPP_COMPONENTS_REGISTER_NODE`

From [`ros2/demos` — jazzy branch](https://github.com/ros2/demos/blob/jazzy/composition/src/talker_component.cpp) (Apache 2.0):

```cpp
// Source: github.com/ros2/demos/blob/jazzy/composition/src/talker_component.cpp
#include "composition/talker_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
using namespace std::chrono_literals;

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
  RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg->data.c_str());
  pub_->publish(std::move(msg));   // move semantics — enables zero-copy IPC path
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Talker)
```

The `RCLCPP_COMPONENTS_REGISTER_NODE` macro generates the factory function the container uses to instantiate the class by plugin name at runtime.

---

## Launch file — static loading

### ComposableNodeContainer

All nodes loaded at startup in a single container:

```python
# Source: github.com/ros2/demos/blob/jazzy/composition/launch/composition_demo_launch.py (Apache 2.0)
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',       # or component_container_mt / _isolated
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Talker',
                name='talker',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name='listener',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return LaunchDescription([container])
```

### LoadComposableNodes — loading into an existing container

Use `LoadComposableNodes` when the container is started separately (e.g. in a base launch file) and you want to add components from a downstream launch file without restarting the container:

```python
# Source: docs.ros.org/en/jazzy/How-To-Guides/Launching-composable-nodes.html
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        # Start a bare container
        Node(
            name='image_container',
            package='rclcpp_components',
            executable='component_container',
            output='both',
        ),
        # Load components into the running container
        LoadComposableNodes(
            target_container='image_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_tools',
                    plugin='image_tools::Cam2Image',
                    name='cam2image',
                    remappings=[('/image', '/burgerimage')],
                    parameters=[{
                        'width': 320,
                        'height': 240,
                        'burger_mode': True,
                        'history': 'keep_last',
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='image_tools',
                    plugin='image_tools::ShowImage',
                    name='showimage',
                    remappings=[('/image', '/burgerimage')],
                    parameters=[{'history': 'keep_last'}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),
    ])
```

`remappings` and `parameters` in `ComposableNode` follow exactly the same format as in the `Node` launch action. The `target_container` is the node name (not the namespace-qualified name) of the container started earlier in the launch graph.

---

## CLI — runtime loading

```bash
# Start a bare container
ros2 run rclcpp_components component_container --ros-args -r __node:=ComponentManager

# Load a component by plugin name
ros2 component load /ComponentManager composition composition::Talker

# Load with a parameter override (-p key:=value)
ros2 component load /ComponentManager image_tools image_tools::Cam2Image \
    -p burger_mode:=true

# Load with intra-process comms enabled (-e for extra/node-options arguments)
ros2 component load /ComponentManager composition composition::Talker \
    -e use_intra_process_comms:=true

# Load with node name and namespace remapping
ros2 component load /ComponentManager composition composition::Talker \
    --node-name talker3 --node-namespace /ns2

# Inspect loaded components (returns component ID and plugin name)
ros2 component list /ComponentManager

# Unload by numeric component ID
ros2 component unload /ComponentManager 1
```

*(Source: [Composing multiple nodes in a single process — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html))*

---

## Intra-process communication (IPC)

### How it works

With IPC enabled, publishers and subscribers in the **same container** exchange `std::unique_ptr<MsgT>` messages through a lock-free ring buffer managed by `rclcpp`'s intra-process manager — bypassing DDS entirely. The publisher transfers ownership of the allocation to the subscriber; no serialisation, no copy, no network round-trip.

IPC is only active when **all** of the following hold:

1. `use_intra_process_comms: true` is set on both the publishing and subscribing node
2. Both nodes are in the **same container process**
3. The publisher uses `std::make_unique<MsgT>()` and `publish(std::move(msg))`
4. The subscriber callback takes `std::unique_ptr<MsgT>` (not `SharedPtr`) — any `SharedPtr` subscriber in the pipeline forces a copy at the handoff
5. Publisher and subscriber have **compatible QoS** (same depth and reliability)

If any condition fails, `rclcpp` silently falls back to DDS transport — no error is raised.

### Verifying IPC is active

The most direct verification is to log the memory address of the message at publish and receive time. If IPC is active, both addresses are identical (ownership was transferred, not copied):

```cpp
// Publisher side
void Talker::on_timer()
{
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "ping";
    RCLCPP_INFO(get_logger(), "Publishing msg at address: %p", (void *)msg.get());
    pub_->publish(std::move(msg));
}

// Subscriber side
void Listener::topic_callback(std::unique_ptr<std_msgs::msg::String> msg)
{
    RCLCPP_INFO(get_logger(), "Received msg at address: %p", (void *)msg.get());
    // If IPC is active, the address matches the publisher's log line
}
```

*(Source: [Intra-Process Communications in ROS 2 — design.ros2.org](https://design.ros2.org/articles/intraprocess_communications.html))*

You can also confirm all components are in the same OS process:
```bash
# Both component PIDs should be identical
ros2 component list /ComponentManager   # lists components by ID
ps aux | grep component_container       # shows the single OS process
```

---

## Diagnostics

```bash
# List all components running in a container (with numeric IDs for unloading)
ros2 component list /ComponentManager

# Confirm node is visible on the ROS graph (same names/topics as standalone)
ros2 node list
ros2 node info /talker

# Check that intra-process subscription is NOT creating a DDS subscription
# (with IPC active, ros2 topic info shows 0 DDS subscribers for topics
# that are only subscribed to intra-process)
ros2 topic info /chatter --verbose
```

If `ros2 topic info --verbose` shows a non-zero subscriber count for a topic you expect to be purely intra-process, IPC fell back to DDS — check the `unique_ptr` / QoS / same-container conditions above.

---

## Common pitfalls

**1. IPC requires `unique_ptr` publish and a `unique_ptr`-taking subscriber.**  
Publishing with `std::make_shared` or having any subscriber in the pipeline that takes `SharedPtr` forces a copy at the handoff point, silently defeating zero-copy. Always `publish(std::move(msg))` with `std::make_unique`, and write subscriber callbacks as `void cb(std::unique_ptr<MsgT> msg)` throughout the IPC-critical path.

**2. Callback groups created after the executor starts don't register.**  
If you dynamically add a callback group inside a component after the container has already started spinning, its callbacks will be silently ignored. Create all callback groups in the constructor. ([rclcpp issue #2067](https://github.com/ros2/rclcpp/issues/2067))

**3. Using `component_container` (single-threaded) for components with blocking callbacks.**  
If one component's callback blocks (service call, `sleep`, `spin_until_future_complete`), all other components in the same container stall. Use `component_container_mt` for shared concurrency, or `component_container_isolated` to give each component its own executor thread.

**4. `component_container_isolated` can saturate CPU under high component count.**  
Each component gets its own dedicated thread. If you load many high-frequency components (camera pipelines, lidars), you can hit CPU saturation before you would with a shared `MultiThreadedExecutor`. Profile with `top -H` or `ros2 run tracetools_analysis` before choosing isolation over sharing. ([rclcpp issue #2295](https://github.com/ros2/rclcpp/issues/2295))

**5. Forgetting `install(TARGETS ... DESTINATION lib)` in CMakeLists.txt.**  
`rclcpp_components_register_nodes` writes the plugin into the ament index using the installed library path. If the `install()` call is missing, `colcon build` succeeds but `ros2 component load` fails with `could not find node plugin` because the shared library is not on the ament package path.

**6. `target_container` in `LoadComposableNodes` must match the node name, not the executable name.**  
The `target_container` argument is the ROS node name of the container (the `name=` field in the `Node` action, or the value passed to `-r __node:=`), not the executable name `component_container`. A mismatch causes `LoadComposableNodes` to time out silently waiting for a container that never responds.

---

## Further reading

- [Composing multiple nodes in a single process — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html) — full tutorial: CLI demo, IPC demo, link flags, all three container types
- [Using ROS 2 launch to launch composable nodes — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Launching-composable-nodes.html) — `ComposableNodeContainer`, `LoadComposableNodes`, `ComposableNode` with parameters and remappings
- [ros2/demos — composition package, jazzy branch (GitHub)](https://github.com/ros2/demos/tree/jazzy/composition) — full C++ source for Talker, Listener, Server, Client components used throughout the official docs
- [rclcpp_components source — ros2/rclcpp, jazzy (GitHub)](https://github.com/ros2/rclcpp/tree/jazzy/rclcpp_components) — component_container_isolated implementation, register macros, NodeFactory API
- [Intra-Process Communications design article — design.ros2.org](https://design.ros2.org/articles/intraprocess_communications.html) — ring buffer design, ownership transfer semantics, when IPC activates vs falls back
- [About Composition — ROS 2 Humble concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) — conceptual overview of the plugin architecture and ament index discovery mechanism

---

*2026-08-15 | ROS2 version: Jazzy / Humble*
