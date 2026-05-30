# ROS2 Nodes — Creating, Composing, and Managing

A node is the fundamental unit of computation in ROS2: an isolated, named process that owns its communication entities (publishers, subscriptions, timers, services, action servers) and is driven by an **Executor**. Getting node design right determines whether your system is debuggable, restartable, and composable — especially when deploying to constrained hardware or safety-critical platforms.

---

## What it is

Every ROS2 node is an instance of `rclcpp::Node` (C++) or `rclpy.node.Node` (Python). A node's fully-qualified name is `/namespace/name`. Two nodes with the same FQN cause the older instance to be killed by the RMW layer — a common source of mysterious restarts in multi-launch systems.

ROS2 ships three increasingly powerful node variants:

| Variant | When to use |
|---|---|
| `rclpy.node.Node` | Stateless data-processing or control nodes |
| `rclpy.lifecycle.Node` | Hardware drivers, I/O nodes, anything needing coordinated startup/shutdown |
| Composable node (`rclcpp_components`) | High-throughput pipelines where cross-process IPC overhead is measurable |

---

## Executors and callback groups

The **Executor** is the thread model that dispatches callbacks. `rclpy.spin()` is shorthand for a `SingleThreadedExecutor` — all callbacks share one thread. This is fine for simple nodes; it deadlocks the moment a callback blocks (sleeps, calls a service synchronously, or calls `spin_until_future_complete()` on its own executor).

Switch to `MultiThreadedExecutor` and assign callbacks to groups to control concurrency:

```python
# Source: docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Callbacks in separate groups run concurrently with each other
        cb_group_a = MutuallyExclusiveCallbackGroup()
        cb_group_b = MutuallyExclusiveCallbackGroup()

        # Sensor callback runs independently from the control callback
        self.sub = self.create_subscription(
            SensorMsg, '/scan', self.sensor_cb, 10,
            callback_group=cb_group_a)
        self.timer = self.create_timer(
            0.05, self.control_cb,
            callback_group=cb_group_b)

    def sensor_cb(self, msg): ...
    def control_cb(self): ...

def main():
    rclpy.init()
    node = MyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
```

**`MutuallyExclusiveCallbackGroup`** — callbacks within the group never execute concurrently (serialized), safe for shared state.  
**`ReentrantCallbackGroup`** — multiple instances of the *same* callback can overlap (e.g. a slow sensor callback while a timer fires); requires the callback to be thread-safe.

**Keep the callback group reference on `self`.** In rclcpp, a callback group assigned to a local variable goes out of scope and the executor silently drops those callbacks — no error, no warning.

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

Override the transition callbacks to acquire and release hardware resources at the right time. Below is the essential pattern from [`ros2/demos`](https://github.com/ros2/demos/blob/humble/lifecycle_py/lifecycle_py/talker.py) (Apache 2.0):

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
        # Create publishers and timers here, not in __init__
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

Transition from outside:
```bash
ros2 lifecycle set /lc_talker configure
ros2 lifecycle set /lc_talker activate
ros2 lifecycle get /lc_talker   # print current state
```

`TransitionCallbackReturn.FAILURE` keeps the node in the current state; `ERROR` transitions to `errorprocessing`. Never raise exceptions from transition callbacks — use the return value.

---

## Composable nodes

Composable nodes (C++ only) run inside a **component container** process, sharing an address space and DDS subscription. This eliminates cross-process serialization overhead — critical for high-bandwidth pipelines (cameras, point clouds). The same component can be loaded at runtime via `ros2 component load` or statically via a launch file.

Launch two components into a shared container with the canonical example from [`ros2/demos`](https://github.com/ros2/demos/blob/rolling/composition/launch/composition_demo_launch.py):

```python
# Source: github.com/ros2/demos — rolling/composition/launch/composition_demo_launch.py

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',          # single-threaded; use component_container_mt for multi-threaded
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Talker',
                name='talker'),
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name='listener'),
        ],
        output='screen',
    )
    return launch.LaunchDescription([container])
```

Enable zero-copy intra-process transport by adding `extra_arguments=[{'use_intra_process_comms': True}]` to each `ComposableNode`. For this to eliminate copies end-to-end, all nodes in the pipeline must be in the same container and use `std::unique_ptr` message ownership in C++.

---

## Common pitfalls

**1. Blocking inside a callback with a single-threaded executor.**
Any `sleep`, synchronous service call, or `spin_until_future_complete()` from inside a callback starves the executor. Use `MultiThreadedExecutor` with a `ReentrantCallbackGroup` for the blocking callback, or restructure to chain futures.

**2. Relying on default node names in multi-robot deployments.**
Two nodes sharing a FQN cause silent kills. Always parameterize the node name in `LaunchDescription` via `arguments=['--ros-args', '-r', '__node:=<unique>']` or check for collisions with `ros2 node list`.

**3. Creating publishers and timers in `__init__` for a `LifecycleNode`.**
These entities must be created in `on_configure()` and destroyed in `on_cleanup()`. Creating them in the constructor means they're active before the state machine permits — you'll publish without an active lifecycle publisher and get silent message drops.

**4. Using `component_container` (single-threaded) for concurrent components.**
If one component's callback blocks, all others in that container stall. Use `component_container_mt` (`rclcpp_components` package) to spin with a `MultiThreadedExecutor`, or load time-sensitive components into separate containers.

---

## Further reading

- [About Executors — ROS 2 Humble docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) — thread model, callback scheduling, real-time considerations
- [Using Callback Groups — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) — complete rclpy and rclcpp examples with concurrency rules
- [Managed Nodes (lifecycle) design article — design.ros2.org](https://design.ros2.org/articles/node_lifecycle.html) — state machine spec, rationale, and transition semantics
- [About Composition — ROS 2 Humble docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html) — composable node architecture, intra-process communication, runtime loading
- [ros2/demos — lifecycle_py (GitHub)](https://github.com/ros2/demos/tree/humble/lifecycle_py) — full lifecycle talker/listener source used in this article

---
*2026-05-30 | ROS2 version: Jazzy / Humble*
