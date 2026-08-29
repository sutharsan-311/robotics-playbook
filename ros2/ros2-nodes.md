# ROS2 Nodes — Executors, Callback Groups, and Spin Patterns

A node is the fundamental unit of computation in ROS 2: a named process that owns publishers, subscriptions, timers, services, and action servers, all driven by an **Executor**. This article covers what actually happens between `rclcpp::init` and `rclcpp::shutdown` — the executor model, callback scheduling, callback groups, thread assignment, and the spin patterns that determine correctness under load.

For lifecycle node state machines and composable node containers, see the dedicated articles in this series.

---

## Node fundamentals

Every ROS 2 node has a **fully-qualified name** (FQN) of the form `/namespace/name`. The FQN is the identity the RMW layer uses — two nodes with the same FQN cause the earlier instance to be killed, silently, with no error on the newer one. In multi-robot systems always parameterize the node name.

### rclpy — minimal node

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
import rclpy
from rclpy.node import Node


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # All create_* calls go here; entities are registered with the executor
        # on the spin() call that follows construction.
        self.pub = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        ...


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

// For composable nodes: construction signature
MyNode::MyNode(const rclcpp::NodeOptions & options)
: Node("my_node", options)
{}
```

`automatically_declare_parameters_from_overrides(true)` is the correct option when you load a YAML parameter file at launch and want all keys to appear as declared parameters without listing each one in the constructor. Without it, keys in the YAML that are not explicitly `declare_parameter`d are silently ignored.

### Remapping at launch time

Node names and namespaces are the most common things remapped in multi-robot deployments:

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

---

## Executor types

The Executor is the thread model that dispatches callbacks. `rclpy.spin()` and `rclcpp::spin()` are shorthand for a `SingleThreadedExecutor`. Four executor types are available in rclcpp on Jazzy:

| Executor | Threading | When to use |
|---|---|---|
| `SingleThreadedExecutor` | 1 thread | Simple nodes; no blocking callbacks |
| `MultiThreadedExecutor` | N threads (default: hardware_concurrency) | Parallel callbacks; nodes with blocking calls isolated to their own callback group |
| `StaticSingleThreadedExecutor` | 1 thread, static callback table | Lower overhead in real-time contexts; all entities must be created before `spin()` |
| `EventsExecutor` *(experimental)* | Event-driven, configurable threads | Lowest latency; callbacks invoked immediately on DDS/timer event, not on the next spin cycle |

### StaticSingleThreadedExecutor — real-time use

The standard `SingleThreadedExecutor` rebuilds its executable list every spin cycle (O(n) in entity count). `StaticSingleThreadedExecutor` builds the list once at startup and does not modify it — this eliminates the per-cycle allocation and is the correct choice for a real-time control loop node where deterministic overhead matters more than dynamic entity creation.

**Constraint:** all publishers, subscriptions, timers, and services must be created before the executor starts spinning. Any entity created after `spin()` begins will not be scheduled.

```cpp
// Source: docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyControlNode>();

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node);

    // Pin this thread to a real-time SCHED_FIFO policy before spinning
    // (use pthread_setschedparam or a wrapper like realtime_tools)
    executor.spin();

    rclcpp::shutdown();
}
```

### EventsExecutor — lowest latency (experimental)

`EventsExecutor` replaces the polling model with an event queue: instead of iterating all ready entities on every spin cycle, callbacks are enqueued the moment the underlying DDS or timer fires and are dispatched immediately.

```cpp
// Source: docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1experimental_1_1executors_1_1EventsExecutor.html
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();

    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
}
```

**Known issue (Jazzy):** `EventsExecutor` can miss timer callbacks after a timer is reset (cancelled and recreated) at high frequency — a pattern used in action server preemption and watchdog resets. Track [rclcpp#2889](https://github.com/ros2/rclcpp/issues/2889) before using in production; validate against your specific workload.

### MultiThreadedExecutor — thread count

```cpp
// Explicit thread count: 4 threads
rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::ExecutorArgs(), 4);
executor.add_node(node);
executor.spin();
```

```python
from rclpy.executors import MultiThreadedExecutor
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

Without the explicit count, both rclpy and rclcpp default to `std::thread::hardware_concurrency()`. On an 8-core machine that's 8 threads — fine for throughput, but all threads compete for CPU which defeats the purpose of isolation. Explicit counts give predictable scheduling.

---

## Spin patterns

### `spin_some` — non-blocking integration

```python
# In an event loop or GUI thread — process all ready callbacks, then return
rclpy.spin_some(node)          # rclpy
executor.spin_some()           # MultiThreadedExecutor
```

```cpp
// rclcpp — process all ready callbacks without blocking
executor.spin_some();

// With a timeout: wait up to 10 ms for work, then return
executor.spin_some(std::chrono::milliseconds(10));
```

Use `spin_some` when the spin loop is external (a game engine, a Qt event loop, a test harness). Call it on every iteration of the external loop. Never use `spin_some` in production code as a substitute for `spin` — it does not block waiting for new work and will burn 100% CPU in a tight loop.

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

**Critical:** never call `spin_until_future_complete` from inside a callback on a `SingleThreadedExecutor`. The executor is already spinning and the second call will either deadlock or throw. On a `MultiThreadedExecutor`, call it only from a callback that is in its own `MutuallyExclusiveCallbackGroup` — so its thread is not blocking the pool.

---

## Callback groups

A callback group controls which callbacks the executor is allowed to execute **concurrently**. On a `SingleThreadedExecutor` all callbacks serialize regardless of group; on a `MultiThreadedExecutor` the groups determine which callbacks can overlap.

| Group type | Within the group | Across different groups |
|---|---|---|
| `MutuallyExclusiveCallbackGroup` | Serialized (at most one active at a time) | Can run concurrently |
| `ReentrantCallbackGroup` | Can overlap freely | Can run concurrently |

Every node has a **default callback group** (MutuallyExclusive) that is used when you don't specify one. Adding a second MutuallyExclusive group is the standard pattern for isolating a slow callback from a fast one.

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

        # Store groups as members — if they go out of scope, callbacks are silently dropped
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

### rclcpp — two isolated groups

```cpp
// Source: docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SensorControlNode : public rclcpp::Node {
public:
    SensorControlNode() : Node("sensor_control_node") {
        // true = auto-add to node's default executor (the default)
        cb_group_sensor_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_control_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_sensor_;

        sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { sensorCb(msg); },
            sub_opts);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { controlCb(); },
            cb_group_control_);
    }

private:
    void sensorCb(sensor_msgs::msg::LaserScan::ConstSharedPtr) { /* heavy work */ }
    void controlCb() { /* 20 Hz control output */ }

    rclcpp::CallbackGroup::SharedPtr cb_group_sensor_;
    rclcpp::CallbackGroup::SharedPtr cb_group_control_;
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

**`MutuallyExclusiveCallbackGroup`** — within the group, at most one callback runs at a time (shared state is safe). Different groups can overlap on a `MultiThreadedExecutor`.

**`ReentrantCallbackGroup`** — multiple invocations of the same callback can run simultaneously. The callback must be thread-safe. Use this for stateless data-transform callbacks, not for callbacks that modify class members.

### Pinning a callback group to a dedicated executor thread

`executor.add_callback_group()` distributes individual callback groups across separate executor instances. This is the pattern for assigning a latency-sensitive callback (e.g. IMU processing) to a real-time thread while keeping everything else on a standard thread:

```cpp
// Source: docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html
auto rt_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);   // false = don't auto-add to the node's default executor

rclcpp::SubscriptionOptions opts;
opts.callback_group = rt_group;
auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10, imu_callback, opts);

// Main executor — owns the node and everything else
rclcpp::executors::SingleThreadedExecutor main_exec;
main_exec.add_node(node);

// Dedicated RT executor — owns only the IMU callback group
rclcpp::executors::StaticSingleThreadedExecutor rt_exec;
rt_exec.add_callback_group(rt_group, node->get_node_base_interface());

// Spin the RT executor on a dedicated thread; set SCHED_FIFO on that thread
std::thread rt_thread([&rt_exec]() {
    rt_exec.spin();
});

main_exec.spin();  // blocks on main thread
rt_thread.join();
```

The `false` argument to `create_callback_group` prevents the group from being auto-registered with the node's default executor. Without it, the callback would be scheduled by **both** executors — double-dispatched, potentially calling the callback concurrently with itself.

---

## Multiple nodes in one executor

An executor can host multiple node instances. This is lighter-weight than component containers for Python pipelines or C++ nodes that don't need zero-copy IPC:

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

Both nodes share the executor's thread pool. Their callbacks are interleaved according to callback group rules. Each node still has its own separate FQN, parameters, and topic namespace.

---

## Common pitfalls

**1. Blocking inside a callback on a `SingleThreadedExecutor`.**
Any `sleep`, synchronous service call via `call()`, or `spin_until_future_complete()` from inside a callback on the same single-threaded executor starves the executor — the executor is blocked inside your callback and cannot service any other callback, including the service response that your callback is waiting for. The result is a deadlock that hangs the node. Switch to `MultiThreadedExecutor` with a dedicated `MutuallyExclusiveCallbackGroup` for the blocking callback, or restructure using `async_send_request` with a future callback.

**2. Letting callback group `shared_ptr` go out of scope.**
`create_callback_group()` returns a `shared_ptr`. If you don't store it as a member variable (`self._cb_group` in Python, `cb_group_` in C++), the group destructs when the constructor returns and the executor silently stops scheduling those callbacks — no error, no warning. Every callback group must be stored for the lifetime of the node.

**3. Creating entities after `StaticSingleThreadedExecutor` starts spinning.**
The static executor builds its entity table once. Any publisher, subscription, timer, or service created after `spin()` is called is invisible to the executor and will never be scheduled. Create all entities in the node constructor before handing the node to the executor.

**4. Calling `rclcpp::spin()` and an executor's `spin()` on the same node.**
`rclcpp::spin(node)` creates a `SingleThreadedExecutor` internally. If you also call `executor.add_node(node)` and `executor.spin()`, two executors compete to service the same node. The result is race conditions in callback dispatch and potential double-invocations. A node must be added to exactly one executor.

**5. FQN collision in multi-robot launch.**
Two nodes with the same `/namespace/name` cause the older instance to be killed by the RMW layer with no warning to the new one. In fleet deployments, always parameterize node names via launch argument and pass `-r __node:=<unique>` or use the `name=` / `namespace=` fields in the `Node` launch action.

**6. `EventsExecutor` with dynamic timer resets.**
A confirmed bug ([rclcpp#2889](https://github.com/ros2/rclcpp/issues/2889)) causes `EventsExecutor` to miss timer callbacks when a timer is reset (cancelled and re-created) at high frequency — a pattern used in action server preemption and watchdog resets. Do not use `EventsExecutor` in nodes that reset timers unless you have explicitly validated against this bug in your ROS 2 version.

**7. `add_callback_group` without `false` in the constructor call.**
When you use `executor.add_callback_group(group, node_base)` to assign a group to a dedicated executor, you must pass `false` as the `automatically_add_to_executor` argument when calling `create_callback_group`. Otherwise the group is auto-registered with the node's default executor **and** added to the dedicated one — two executors compete to dispatch the same callbacks.

---

## Further reading

- [About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html) — polling model, callback table, `StaticSingleThreadedExecutor`, real-time considerations
- [Using Callback Groups — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) — complete rclpy and rclcpp examples with concurrency rules and `add_callback_group`
- [Class EventsExecutor — rclcpp Jazzy API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1experimental_1_1executors_1_1EventsExecutor.html) — event-driven executor API reference; still experimental in Jazzy
- [Class NodeOptions — rclcpp Jazzy API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1NodeOptions.html) — full option list: use_global_arguments, allow_undeclared_parameters, automatically_declare_parameters_from_overrides, and more
- [ros2/examples — rclpy executors (GitHub)](https://github.com/ros2/examples/tree/rolling/rclpy/executors) — callback group and multi-node executor patterns used in this article
- [Deadlocks in rclpy and how to prevent them — Karelics](https://karelics.fi/deadlocks-in-rclpy/) — detailed walkthrough of every deadlock pattern in single- and multi-threaded executors
- [The Evolution of Execution Management in rclcpp — Polymath Robotics](https://www.polymathrobotics.com/blog/execution-management-in-rclcpp) — deep dive into why the polling model was replaced and what EventsExecutor changes
- [ROS 2 Nodes — Lifecycle variant](ros2-lifecycle-nodes.md) — state machine, transition callbacks, configure/activate/deactivate/cleanup
- [ROS 2 Component Nodes](ros2-component-nodes.md) — composable executors, intra-process comms, zero-copy IPC

---

*2026-08-29 | ROS2 version: Jazzy / Humble*
