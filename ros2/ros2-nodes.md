# ROS2 Nodes — Creating, Composing, and Managing

A node is the fundamental unit of computation in ROS2: an isolated, named process that owns its communication entities (publishers, subscriptions, timers, services, action servers) and is driven by an **Executor**. Getting node design right determines whether your system is debuggable, restartable, and composable — especially when deploying to constrained hardware or safety-critical platforms.

---

## What it is

Every ROS2 node is an instance of `rclcpp::Node` (C++) or `rclpy.node.Node` (Python). A node's fully-qualified name is `/namespace/name`. Two nodes with the same FQN cause the older instance to be killed by the RMW layer — a common source of mysterious restarts in multi-launch systems.

ROS2 ships three increasingly powerful node variants:

| Variant | When to use |
|---|---|
| `rclpy.node.Node` / `rclcpp::Node` | Stateless data-processing or control nodes |
| `rclpy.lifecycle.Node` / `rclcpp_lifecycle::LifecycleNode` | Hardware drivers, I/O nodes, anything needing coordinated startup/shutdown |
| Composable node (`rclcpp_components`) | High-throughput pipelines where cross-process IPC overhead is measurable |

---

## Executors and callback groups

The **Executor** is the thread model that dispatches callbacks. `rclpy.spin()` is shorthand for a `SingleThreadedExecutor` — all callbacks share one thread. This is fine for simple nodes; it deadlocks the moment a callback blocks (sleeps, calls a service synchronously, or calls `spin_until_future_complete()` on its own executor).

Three executor types are available in both rclcpp and rclpy:

| Executor | Threading | When to use |
|---|---|---|
| `SingleThreadedExecutor` | 1 thread | Simple nodes; no blocking callbacks |
| `MultiThreadedExecutor` | N threads (default: CPU count) | Nodes with parallel callbacks or blocking calls |
| `StaticSingleThreadedExecutor` | 1 thread, static callback table | Lower overhead in real-time contexts; all entities must be created before `spin()` |

Switch to `MultiThreadedExecutor` and assign callbacks to groups to control concurrency:

### rclpy

```python
# Pattern from ros2/examples — rclpy/executors/examples_rclpy_executors/callback_group.py
# (Apache 2.0, https://github.com/ros2/examples)
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SensorControlNode(Node):
    def __init__(self):
        super().__init__('sensor_control_node')

        # Two separate groups → the scan callback and control timer run concurrently
        cb_group_sensor = MutuallyExclusiveCallbackGroup()
        cb_group_control = MutuallyExclusiveCallbackGroup()

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.sensor_cb, 10,
            callback_group=cb_group_sensor)
        self.timer = self.create_timer(
            0.05, self.control_cb,
            callback_group=cb_group_control)

    def sensor_cb(self, msg: LaserScan): ...
    def control_cb(self): ...


def main():
    rclpy.init()
    node = SensorControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
```

### rclcpp

```cpp
// Source: docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SensorControlNode : public rclcpp::Node {
public:
    SensorControlNode() : Node("sensor_control_node") {
        auto cb_group_sensor = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto cb_group_control = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_sensor;

        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { sensorCb(msg); },
            sub_options);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { controlCb(); },
            cb_group_control);
    }

private:
    void sensorCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) { /* ... */ }
    void controlCb() { /* ... */ }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
```

**`MutuallyExclusiveCallbackGroup`** — callbacks within the group never execute concurrently (serialized), safe for shared state.  
**`ReentrantCallbackGroup`** — multiple instances of the *same* callback can overlap (e.g. a slow sensor callback while a timer fires); requires the callback to be thread-safe.

**Keep the callback group reference on `self` / as a member variable.** In rclcpp, a callback group assigned to a local variable goes out of scope and the executor silently drops those callbacks — no error, no warning. In rclpy the same applies.

Since Galactic, rclcpp also exposes `executor.add_callback_group(group, node->get_node_base_interface())`, allowing you to distribute individual callback groups across separate executors — useful for pinning time-sensitive callbacks to a dedicated real-time thread.

---

## Lifecycle nodes

`LifecycleNode` adds a state machine that gives external tools (systemd, fleet managers, launch files) explicit control over startup and shutdown ordering. The states are:

```
unconfigured
     │ configure()
   inactive
     │ activate()
   active ←→ deactivate() → inactive
     │ shutdown() / cleanup()
  finalized
```

Override the transition callbacks to acquire and release hardware resources at the right time. The example below is the canonical pattern from [`ros2/demos`](https://github.com/ros2/demos/blob/humble/lifecycle_py/lifecycle_py/talker.py) (Apache 2.0):

```python
# Source: github.com/ros2/demos — humble/lifecycle_py/lifecycle_py/talker.py
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
import rclpy
import std_msgs.msg


class LifecycleTalker(Node):

    def __init__(self, node_name, **kwargs):
        self._pub = None
        self._timer = None
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Create publishers and timers here, NOT in __init__
        self._pub = self.create_lifecycle_publisher(
            std_msgs.msg.String, 'lifecycle_chatter', 10)
        self._timer = self.create_timer(1.0, self.publish)
        self.get_logger().info('on_configure() called')
        return TransitionCallbackReturn.SUCCESS   # → inactive

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate() called')
        return super().on_activate(state)         # activates the LifecyclePublisher

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        return super().on_deactivate(state)       # deactivates the LifecyclePublisher

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)
        return TransitionCallbackReturn.SUCCESS   # → unconfigured

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self.destroy_publisher(self._pub)
        return TransitionCallbackReturn.SUCCESS   # → finalized

    def publish(self):
        if self._pub and self._pub.is_activated:
            msg = std_msgs.msg.String(data='Hello')
            self._pub.publish(msg)


def main():
    rclpy.init()
    node = LifecycleTalker('lc_talker')
    rclpy.spin(node)
```

Trigger transitions from the CLI:
```bash
ros2 lifecycle set /lc_talker configure
ros2 lifecycle set /lc_talker activate
ros2 lifecycle get /lc_talker   # print current state
```

`TransitionCallbackReturn.FAILURE` keeps the node in the current state; `ERROR` transitions to `errorprocessing`. Never raise exceptions from transition callbacks — use the return value.

---

## Composable nodes

Composable nodes (C++ only) run inside a **component container** process, sharing an address space and DDS subscription. This eliminates cross-process serialization overhead — critical for high-bandwidth pipelines (cameras, point clouds). The same component can be loaded at runtime via `ros2 component load` or statically via a launch file.

### Registering a component

Every composable node must register itself with the component system using the macro from `rclcpp_components`:

```cpp
// my_component.cpp
#include "rclcpp_components/register_node_macro.hpp"
#include "my_pkg/my_component.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MyComponent)
```

The macro generates the factory function the container uses to instantiate your node. The class must also expose a constructor taking `const rclcpp::NodeOptions &`.

### Launching components into a shared container

```python
# Source: docs.ros.org/en/humble/How-To-Guides/Launching-composable-nodes.html
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',   # multi-threaded; use component_container for single-threaded
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Talker',
                name='talker',
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name='listener',
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
```

For zero-copy intra-process transport to work end-to-end, all nodes in the pipeline must be in the same container **and** use `std::unique_ptr<MsgT>` message ownership in C++. Mixed `SharedPtr`/`UniquePtr` ownership in the same pipeline re-enables copies at the handoff point.

Load or unload a component at runtime without stopping the container:
```bash
ros2 component load /my_container composition composition::Talker
ros2 component list
ros2 component unload /my_container 1
```

---

## Common pitfalls

**1. Blocking inside a callback with a single-threaded executor.**  
Any `sleep`, synchronous service call, or `spin_until_future_complete()` from inside a callback starves the executor. Use `MultiThreadedExecutor` with separate `MutuallyExclusiveCallbackGroup`s for the blocking callback, or restructure to chain futures.

**2. Relying on default node names in multi-robot deployments.**  
Two nodes sharing a FQN cause silent kills. Always parameterize the node name in `LaunchDescription` via `arguments=['--ros-args', '-r', '__node:=<unique>']` or check for collisions with `ros2 node list`.

**3. Creating publishers and timers in `__init__` for a `LifecycleNode`.**  
These entities must be created in `on_configure()` and destroyed in `on_cleanup()`. Creating them in the constructor means they are active before the state machine permits — you'll publish without an active lifecycle publisher and get silent message drops.

**4. Using `component_container` (single-threaded) for concurrent components.**  
If one component's callback blocks, all others in that container stall. Use `component_container_mt` to spin with a `MultiThreadedExecutor`, or load time-sensitive components into separate containers.

**5. Letting callback group handles go out of scope (rclcpp).**  
`create_callback_group()` returns a `shared_ptr`. If you don't store it as a member, the group is destroyed and the executor silently stops scheduling those callbacks. Store every group as a named member variable.

---

## Further reading

- [About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html) — thread model, callback scheduling, StaticSingleThreadedExecutor, real-time considerations
- [Using Callback Groups — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) — complete rclpy and rclcpp examples with concurrency rules
- [Deadlocks in rclpy and how to prevent them — Karelics](https://karelics.fi/deadlocks-in-rclpy/) — detailed walkthrough of every deadlock pattern in single- and multi-threaded executors
- [Managed Nodes (lifecycle) design article — design.ros2.org](https://design.ros2.org/articles/node_lifecycle.html) — state machine spec, rationale, and transition semantics
- [Writing a Composable Node (C++) — ROS 2 Humble docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-a-Composable-Node.html) — CMakeLists, register macro, NodeOptions constructor pattern
- [ros2/demos — lifecycle_py (GitHub)](https://github.com/ros2/demos/tree/humble/lifecycle_py) — full lifecycle talker/listener source used in this article
- [ros2/examples — rclpy executors (GitHub)](https://github.com/ros2/examples/tree/rolling/rclpy/executors) — callback group and executor patterns used in this article

---
*2026-06-13 | ROS2 version: Jazzy / Humble*
