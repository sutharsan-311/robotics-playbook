# ROS2 Nodes — Lifecycle, Naming, Introspection, and Spin Patterns

A node is the fundamental unit of computation in ROS 2: a named process that owns publishers, subscriptions, timers, services, and action servers, all driven by an Executor. This article covers what a practicing engineer needs to know about node construction, naming and remapping, NodeOptions, introspection CLI tools, and spin patterns. For a deep dive into executor selection, callback groups, and the EventsExecutor, see [ros2-executors.md](ros2-executors.md).

---

## Node fundamentals

Every ROS 2 node has a **fully-qualified name** (FQN) of the form `/namespace/name`. The FQN is the identity the RMW layer uses for discovery and graph introspection. Two nodes sharing the same FQN cause the **earlier** instance to be killed silently — no warning is emitted by the node being started. In multi-robot systems always parameterize the node name at launch.

### rclpy — minimal node

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # All create_* calls go here; entities are registered with the executor
        # on the spin() call that follows construction.
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### rclcpp — NodeOptions

`NodeOptions` is the C++ equivalent of keyword arguments to `Node.__init__`. The three most important flags:

| Option | Default | What it controls |
|--------|---------|-----------------|
| `use_global_arguments` | `true` | Whether CLI args not targeted at a specific node affect this node |
| `allow_undeclared_parameters` | `false` | Whether `set_parameter` on an undeclared parameter throws or succeeds |
| `automatically_declare_parameters_from_overrides` | `false` | Whether parameters from a YAML/override file are auto-declared on startup |

```cpp
// Source: github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/node_options.hpp
rclcpp::NodeOptions options;
options.use_global_arguments(true)
       .allow_undeclared_parameters(false)
       .automatically_declare_parameters_from_overrides(true);

// Composable node construction signature — required for component containers
MyNode::MyNode(const rclcpp::NodeOptions & options)
: Node("my_node", options)
{}
```

`automatically_declare_parameters_from_overrides(true)` is the correct option when loading a YAML parameter file at launch: all keys in the YAML appear as declared parameters without listing each one in the constructor. Without it, keys not explicitly `declare_parameter`d are silently ignored.

*(Source: [Class NodeOptions — rclcpp Jazzy API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html))*

---

## Remapping at launch time

Node names, namespaces, and topics are the most common things remapped in multi-robot deployments:

```bash
# CLI: remap node name and a topic in the same invocation
ros2 run my_pkg my_node \
    --ros-args \
    -r __node:=robot1_controller \
    -r __ns:=/robot1 \
    -r /cmd_vel:=/robot1/cmd_vel

# Launch file equivalent
from launch_ros.actions import Node

Node(
    package='my_pkg',
    executable='my_node',
    name='robot1_controller',
    namespace='robot1',
    remappings=[('/cmd_vel', '/robot1/cmd_vel')],
)
```

`-r __node:=` and `-r __ns:=` are ROS 2 magic remapping arguments — they change the node's FQN before the node constructor runs. Topic remappings follow the same `source:=target` format.

> **Multi-robot tip:** combine `namespace=` and a robot ID parameter so the same launch file brings up an entire fleet with one `OpaqueFunction` loop. The namespace scopes every topic, service, and action the node owns without touching the node's source code.

*(Source: [About ROS 2 remapping — design.ros2.org](https://design.ros2.org/articles/ros_command_line_arguments.html))*

---

## Executor types — quick reference

The Executor is the thread model that dispatches callbacks. For full coverage of each type, pitfalls, and real benchmarks, see [ros2-executors.md](ros2-executors.md). Quick reference:

| Executor | Threading | Jazzy / Rolling status |
|---|---|---|
| `SingleThreadedExecutor` | 1 thread | ✅ Stable |
| `MultiThreadedExecutor` | N threads | ✅ Stable |
| `StaticSingleThreadedExecutor` | 1 thread, static callback table | ⚠️ **Deprecated in Jazzy, removed in Rolling** |
| `EventsExecutor` *(experimental)* | Event-driven | `rclcpp::experimental::executors` — Jazzy+ |

> **`StaticSingleThreadedExecutor` — migration required.** This executor was deprecated in Jazzy when its optimisations were upstreamed into the standard executors, and it has been removed entirely in Rolling. Any Humble codebase using `StaticSingleThreadedExecutor` should migrate to `SingleThreadedExecutor`. The standard executor performs comparably on fixed node graphs and does not require all entities to be created before `spin()`. ([About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html))

**For real-time control** in Jazzy, the correct pattern is `SingleThreadedExecutor` with `SCHED_FIFO` thread pinning via `pthread_setschedparam` or the `realtime_tools` package — not `StaticSingleThreadedExecutor`. Alternatively, `EventsExecutor` eliminates the polling model entirely and shows 75% lower idle CPU and latency, but is still under `experimental::executors` and should be validated against your specific workload.

```cpp
// Real-time: pin the executor thread to SCHED_FIFO after construction
#include "rclcpp/rclcpp.hpp"
#include <pthread.h>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyControlNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Pin to SCHED_FIFO priority 80 before spinning — do NOT call this from inside spin()
    sched_param sch;
    sch.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        RCLCPP_WARN(node->get_logger(), "Failed to set SCHED_FIFO — run as root or set RLIMIT_RTPRIO");
    }

    executor.spin();
    rclcpp::shutdown();
}
```

*(Source: [About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html); [realtime_tools — ros-controls/realtime_tools](https://github.com/ros-controls/realtime_tools))*

---

## Spin patterns

### `spin_some` — non-blocking integration

```python
# In an event loop or GUI thread — process all ready callbacks, then return
rclpy.spin_some(node)          # rclpy free function
executor.spin_some()           # preferred when node is attached to an executor
```

```cpp
// rclcpp — process all ready callbacks without blocking
executor.spin_some();

// With a timeout: wait up to 10 ms for work, then return
executor.spin_some(std::chrono::milliseconds(10));
```

Use `spin_some` when the spin loop is external (a game engine, a Qt event loop, a test harness). Call it on every iteration of the external loop. **Never** use `spin_some` as a substitute for `spin` in a standalone node — it does not block waiting for new work and will burn 100% CPU in a tight loop.

### `spin_until_future_complete` — async service calls

The canonical pattern for a one-shot service call from a non-callback context:

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
response = future.result()
```

```cpp
// Source: docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
auto future = client->async_send_request(request);
if (rclcpp::spin_until_future_complete(node, future) ==
    rclcpp::FutureReturnCode::SUCCESS)
{
    auto response = future.get();
}
```

**Critical:** never call `spin_until_future_complete` from inside a callback on a `SingleThreadedExecutor`. The executor is already spinning; the nested call deadlocks immediately. On a `MultiThreadedExecutor`, call it only from a callback in its own `MutuallyExclusiveCallbackGroup`.

---

## Callback groups — essential pattern

A callback group controls which callbacks the executor is allowed to execute concurrently. On a `SingleThreadedExecutor` all callbacks serialize regardless of group; on a `MultiThreadedExecutor` the groups determine which callbacks can overlap.

| Group type | Within the group | Across different groups |
|---|---|---|
| `MutuallyExclusiveCallbackGroup` | Serialized (at most one active at a time) | Can run concurrently |
| `ReentrantCallbackGroup` | Can overlap freely | Can run concurrently |

> **Critical default:** every node uses `MutuallyExclusiveCallbackGroup` for all entities it owns. A `MultiThreadedExecutor` with all callbacks in the default group is functionally identical to `SingleThreadedExecutor`.

### rclpy — two isolated groups

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

        # Two separate groups → scan callback and control timer run concurrently
        cb_group_sensor = MutuallyExclusiveCallbackGroup()
        cb_group_control = MutuallyExclusiveCallbackGroup()

        # IMPORTANT: store groups as members — if they go out of scope,
        # callbacks are silently dropped with no error
        self._cb_groups = (cb_group_sensor, cb_group_control)

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

For full C++ callback group examples, the `add_callback_group` API for dedicating a group to a separate real-time executor, and all callback group pitfalls, see [ros2-executors.md](ros2-executors.md).

---

## Multiple nodes in one executor

An executor can host multiple node instances — lightweight for Python pipelines or C++ nodes that don't need zero-copy IPC:

```python
# Source: ros2/examples — rclpy/executors/examples_rclpy_executors/composed.py (Apache 2.0)
rclpy.init()
node1 = MyNode('node_a')
node2 = MyNode('node_b')

executor = MultiThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()

executor.shutdown()
node1.destroy_node()
node2.destroy_node()
rclpy.shutdown()
```

```cpp
rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);   // 4 threads
executor.add_node(node_a);
executor.add_node(node_b);
executor.spin();
```

Both nodes share the executor's thread pool. Each node has its own FQN, parameters, and topic namespace. Callbacks from different nodes interleave freely — node boundaries imply no thread safety; protect any shared state with a mutex.

---

## Node introspection — CLI tools

These commands are the first line of diagnosis for missing topics, crashed nodes, and broken service connections:

```bash
# List all live nodes in the ROS graph
ros2 node list
# /my_robot/base_controller
# /my_robot/lidar_driver
# /rviz2

# Full node info: publishers, subscriptions, services, action servers, parameters
ros2 node info /my_robot/base_controller
# /my_robot/base_controller
#   Subscribers:
#     /cmd_vel: geometry_msgs/msg/Twist
#   Publishers:
#     /odom: nav_msgs/msg/Odometry
#   Service Servers:
#     /my_robot/base_controller/describe_parameters: rcl_interfaces/srv/DescribeParameters
#   Action Servers:
#     (none)

# Verify a specific node's parameter values at runtime
ros2 param get /my_robot/base_controller wheel_separation
# Double value is: 0.3

# Dump all parameters as YAML (useful for generating launch configs from a live system)
ros2 param dump /my_robot/base_controller

# Kill a specific node cleanly (sends SIGINT, triggers rclcpp shutdown hooks)
ros2 lifecycle set /my_robot/base_controller shutdown  # for lifecycle nodes
kill -SIGINT $(ros2 node list | head -1 | xargs -I{} ros2 node info {} | grep PID)

# List nodes and their process IDs (requires ros2_tracing or --all flag on Jazzy)
ros2 node list --all
```

`ros2 node info` is the first command to run when a subscription or service is silently receiving nothing — it shows whether the node has the expected publisher/subscriber at all, independent of QoS or network issues.

### Programmatic node info — rclcpp

```cpp
// Enumerate all nodes visible in the ROS graph from inside a node
auto node_names = node->get_node_names();            // std::vector<std::string>
auto topic_names = node->get_topic_names_and_types(); // std::map<std::string, std::vector<std::string>>

// Subscribe to node graph change events — fires when any node appears or disappears
auto graph_event = node->get_graph_event();
node->wait_for_graph_change(graph_event, std::chrono::seconds(5));
if (graph_event->check()) {
    RCLCPP_INFO(node->get_logger(), "Graph changed — re-querying node list");
}
```

*(Source: [rclcpp NodeGraph interface — Jazzy API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1Node.html))*

---

## Common pitfalls

**1. Blocking inside a callback on a `SingleThreadedExecutor`.**
Any `sleep`, synchronous service call via `call()`, or `spin_until_future_complete()` from inside a callback on the same single-threaded executor starves the executor — it is blocked inside your callback and cannot service any other callback, including the service response your callback is waiting for. The result is a deadlock. Switch to `MultiThreadedExecutor` with a dedicated `MutuallyExclusiveCallbackGroup` for the blocking callback, or restructure using `async_send_request` with a future callback.

**2. Letting callback group `shared_ptr` go out of scope.**
`create_callback_group()` returns a `shared_ptr`. If you don't store it as a member variable (`self._cb_group` in Python, `cb_group_` in C++), the group destructs when the constructor returns and the executor silently stops scheduling those callbacks — no error, no warning. Every callback group must be stored for the lifetime of the node.

**3. `StaticSingleThreadedExecutor` in Jazzy or Rolling.**
This executor is **deprecated in Jazzy and removed in Rolling**. Code that uses it on Rolling won't compile. Migrate to `SingleThreadedExecutor`. For real-time use, add `SCHED_FIFO` thread pinning to a standard executor instead — the optimisations that motivated `StaticSingleThreadedExecutor` were upstreamed.

**4. FQN collision in multi-robot launch.**
Two nodes with the same `/namespace/name` cause the older instance to be killed by the RMW layer with no warning on either side. In fleet deployments, always parameterize node names via launch argument and pass `-r __node:=<unique>` or use the `name=` / `namespace=` fields in the `Node` launch action.

**5. Calling `rclcpp::spin()` and an executor's `spin()` on the same node.**
`rclcpp::spin(node)` creates a `SingleThreadedExecutor` internally. If you also call `executor.add_node(node)` and `executor.spin()`, two executors compete to service the same node — race conditions in callback dispatch and potential double-invocations. A node must be added to exactly one executor.

**6. Calling `rclpy.spin_once(node)` on a node already attached to an executor.**
The free function `rclpy.spin_once(node)` removes and re-attaches the node internally, breaking its connection to any executor it is already part of. Call `executor.spin_once()` on the executor object instead. ([rclpy issue #1445](https://github.com/ros2/rclpy/issues/1445))

**7. `automatically_declare_parameters_from_overrides` not set when loading YAML.**
Parameters in a YAML file loaded via `--params-file` or `ros__parameters:` in a launch file are only visible if the node has declared them — either explicitly via `declare_parameter()` or automatically via `automatically_declare_parameters_from_overrides(true)` in `NodeOptions`. Without it, YAML keys are silently ignored. This causes `get_parameter()` to throw `ParameterNotDeclaredException` at runtime even though the YAML was correctly specified.

---

## Further reading

- [About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html) — polling model, callback table, `StaticSingleThreadedExecutor` deprecation notice, real-time considerations
- [Using Callback Groups — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) — complete rclpy and rclcpp examples with concurrency rules and `add_callback_group` for dedicated RT threads
- [Class NodeOptions — rclcpp Jazzy API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html) — full option list: use_global_arguments, allow_undeclared_parameters, automatically_declare_parameters_from_overrides, and more
- [About ROS 2 remapping — design.ros2.org](https://design.ros2.org/articles/ros_command_line_arguments.html) — full remapping grammar: `__node`, `__ns`, topic/service/parameter remaps
- [ros2/examples — rclpy executors (GitHub)](https://github.com/ros2/examples/tree/rolling/rclpy/executors) — callback group and multi-node executor patterns used in this article
- [realtime_tools — ros-controls/realtime_tools (GitHub)](https://github.com/ros-controls/realtime_tools) — `RealtimePublisher`, `RealtimeClock`, and thread priority helpers for ROS 2 real-time control
- [Deadlocks in rclpy and how to prevent them — Karelics](https://karelics.fi/deadlocks-in-rclpy/) — detailed walkthrough of every deadlock pattern in single- and multi-threaded executors
- [ros2-executors.md](ros2-executors.md) — deep dive into executor selection, EventsExecutor benchmarks, spin method reference, and all callback group pitfalls

---

*2026-10-17 | ROS2 version: Jazzy / Humble (StaticSingleThreadedExecutor removed in Rolling)*
