# ROS2 Executors — Single, Multi-threaded, Static, Events

An executor is the scheduling engine that drives callback execution in ROS2. Choosing the wrong one — or misconfiguring callback groups alongside it — is one of the most common sources of silent single-threading, subtle deadlocks, and high idle CPU usage in production ROS2 systems.

---

## What it is

Every `rclcpp` or `rclpy` node needs something to call its callbacks: subscription handlers, timers, service servers, action servers. That something is an **executor**. It owns a thread pool (or a single thread), polls the underlying DDS middleware for ready events, and dispatches callbacks to whichever thread is available.

ROS2 ships four executors across Humble and Jazzy:

| Executor | Threads | Status |
|---|---|---|
| `SingleThreadedExecutor` | 1 | Default; stable |
| `MultiThreadedExecutor` | N (configurable) | Stable |
| `StaticSingleThreadedExecutor` | 1 | **Deprecated Jazzy, removed Rolling** |
| `EventsExecutor` | 1 (+ optional timer thread) | Experimental — `rclcpp::experimental::executors` |

**`SingleThreadedExecutor`** serialises all callbacks on one thread in priority order: timers → subscriptions → services → clients. No data races inside callbacks; the safe default for most nodes.

**`MultiThreadedExecutor`** creates a configurable thread pool. Actual parallelism is gated by **callback groups** — simply switching executor type without assigning groups does nothing for throughput (see pitfalls).

**`StaticSingleThreadedExecutor`** skipped rebuilding the wait-set on every iteration by caching entities at `add_node()` time. Its optimisations were upstreamed into the standard executors; the type is formally deprecated in Jazzy and removed in Rolling. Replace with `SingleThreadedExecutor` in any Humble codebase that still uses it.

**`EventsExecutor`** replaces the polling wait-set model with an event-driven queue. Rather than repeatedly calling into the middleware layer to ask "is anything ready?", entities push events onto a lock-free queue as they become ready; the executor drains the queue. Benchmarks from irobot-ros show **~75% lower CPU** and **~75% lower latency** versus `SingleThreadedExecutor` on equivalent workloads. It lives under `rclcpp::experimental::executors::EventsExecutor` (available in Jazzy and Rolling; API may change before stabilisation).

*(Source: [About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html); [irobot-ros/events-executor — GitHub](https://github.com/irobot-ros/events-executor))*

---

## Callback groups

Callback groups are the second axis of executor control. Every entity (subscription, timer, service, client, action) belongs to exactly one callback group. Two types exist:

| Type | Concurrency |
|---|---|
| `MutuallyExclusiveCallbackGroup` | At most one callback in the group runs at a time |
| `ReentrantCallbackGroup` | Multiple callbacks — and multiple instances of the same callback — may run concurrently |

> **Critical default:** a node's built-in default callback group is `MutuallyExclusive`. Using `MultiThreadedExecutor` without assigning any explicit callback groups is functionally identical to `SingleThreadedExecutor` — all callbacks still serialise.

*(Source: [Using Callback Groups — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html))*

### Python — explicit group assignment

From [`ros2/examples` — `callback_group.py`, rolling branch](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/callback_group.py) (Apache 2.0):

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        # Reentrant: multiple subscription callbacks may fire in parallel
        self.group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            String, 'chatter', self.chatter_callback, 10,
            callback_group=self.group)

    def chatter_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    with rclpy.init(args=args):
        node = Listener()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
```

### C++ — explicit group assignment

```cpp
// Source: docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Two groups: serial for timer, reentrant for subscription
    timer_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MyNode::timer_cb, this),
      timer_group_);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = sub_group_;
    sub_ = create_subscription<std_msgs::msg::String>(
      "/chatter", 10,
      std::bind(&MyNode::sub_cb, this, std::placeholders::_1),
      opts);
  }

private:
  void timer_cb() { RCLCPP_INFO(get_logger(), "timer fired"); }
  void sub_cb(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "got: %s", msg->data.c_str());
  }

  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr sub_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);   // 4 threads
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
```

---

## Spin methods

### rclcpp

| Method | Behaviour |
|---|---|
| `rclcpp::spin(node)` | Free function; blocks forever; internally creates a `SingleThreadedExecutor` |
| `executor.spin()` | Blocks forever — standard production form |
| `executor.spin_some(max_duration)` | Executes all *immediately available* work; returns when queue is empty or duration expires. Non-blocking if no work is ready. |
| `executor.spin_once(timeout_ns)` | Executes at most one ready callback; blocks up to `timeout_ns` nanoseconds waiting for one |
| `rclcpp::spin_until_future_complete(node, future)` | Spins until the given `std::shared_future` is complete; free function, creates a temporary executor |

`spin_some()` is the right choice for manual event loops (e.g., a robot main loop that also renders a GUI). Call it with a short deadline rather than calling `spin_once()` in a tight loop — `spin_some()` drains all pending work in one call.

### rclpy

| Method | Behaviour |
|---|---|
| `rclpy.spin(node)` | Free function; blocks forever |
| `executor.spin()` | Blocks forever |
| `executor.spin_once(timeout_sec=None)` | Executes at most one callback; blocks up to `timeout_sec` seconds |
| `executor.spin_until_future_complete(future, timeout_sec=None)` | Blocks until `future.done()` returns `True` or timeout |

*(Source: [rclpy.executors module — rclpy 3.2.1, Jazzy](https://docs.ros.org/en/ros2_packages/jazzy/api/rclpy/rclpy.executors.html))*

> **`rclpy.spin_once()` side-effect:** calling the free function `rclpy.spin_once(node)` on a node that is already attached to an executor internally removes and re-adds the node, breaking its connection to that executor. Prefer calling `executor.spin_once()` on the executor object directly. ([rclpy issue #1445](https://github.com/ros2/rclpy/issues/1445))

---

## Multi-node executors

A single executor can own multiple nodes:

```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node_a)
executor.add_node(node_b)
executor.spin()
```

```cpp
rclcpp::executors::MultiThreadedExecutor executor(
  rclcpp::ExecutorOptions(), 4);
executor.add_node(node_a);
executor.add_node(node_b);
executor.spin();
```

Callbacks from different nodes interleave freely — the executor does not distinguish node ownership when dispatching. If nodes share any state, protect it with a mutex; don't assume node boundaries imply thread safety.

---

## EventsExecutor (experimental, Jazzy / Rolling)

```cpp
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

auto executor =
  std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
executor->add_node(node);
executor->spin();
```

*(Source: [Class EventsExecutor — rclcpp Rolling API](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/classrclcpp_1_1experimental_1_1executors_1_1EventsExecutor.html))*

Key behavioural differences from polling executors:

- **No busy-wait.** The thread sleeps on the queue until an event arrives. Idle CPU drops to near zero versus `SingleThreadedExecutor`'s polling loop.
- **Timer thread option.** Pass `EventsExecutor(std::make_shared<rclcpp::experimental::executors::TimersManager>())` to process timers on a dedicated thread, keeping timer jitter independent of callback backpressure.
- **Same callback-group semantics.** `MutuallyExclusive` and `Reentrant` groups work identically to polling executors.
- **Not yet in stable API.** The header path and class name are under `experimental::executors`. Expect changes before it graduates to `rclcpp::executors`.

The `rosbag2` recorder switched to the related `EventsCBGExecutor` and measured **15–23% less CPU** on average in production benchmarks. ([rosbag2 PR #2472](https://github.com/ros2/rosbag2/pull/2472))

---

## Python MultiThreadedExecutor and the GIL

In CPython, the Global Interpreter Lock (GIL) ensures only one thread executes Python bytecode at any instant. `MultiThreadedExecutor` creates real OS threads, but CPU-bound Python callbacks get **zero parallelism** — they take turns holding the GIL.

`MultiThreadedExecutor` is useful in Python only when callbacks release the GIL:
- **I/O-bound work**: socket reads, file I/O
- **Service/action calls** that block on a future (releases GIL during the wait)
- **C extension calls** that explicitly release the GIL (numpy array ops, OpenCV, TF lookups via `tf2_ros.Buffer`)

For CPU-bound Python workloads (image processing, ML inference), use `multiprocessing` or offload to a C++ component node.

---

## Common pitfalls

**1. `MultiThreadedExecutor` that still runs single-threaded.**  
The default callback group is `MutuallyExclusive`. Adding more threads to the executor without assigning `ReentrantCallbackGroup` or separate `MutuallyExclusive` groups to independent callbacks serialises everything. Verify with `htop -H` that multiple threads are actually busy.

**2. Deadlock: synchronous service/action call inside a callback in the same `MutuallyExclusive` group.**  
A timer callback calls `client->async_send_request()` then waits for the result. The response callback is in the same `MutuallyExclusive` group. The timer holds the group lock; the response callback can never acquire it. Fix: put the request-sending callback and the response-receiving callback in *different* callback groups, or use a `ReentrantCallbackGroup` for both, or avoid synchronous waits entirely. ([rclcpp issue #773](https://github.com/ros2/rclcpp/issues/773))

**3. `spin_until_future_complete` may block forever after the future resolves.**  
If the future is completed by code outside the executor's callback path, no new event wakes the executor thread after the future's done-callback fires — the spin blocks indefinitely. Always set a `timeout_sec` / `timeout_ns`. ([rclcpp issue #1916](https://github.com/ros2/rclcpp/issues/1916))

**4. Callback groups created after `executor.spin()` starts are silently ignored.**  
`rclcpp` registers callback groups with the executor's wait-set at node-add time. Groups created dynamically after spinning has begun are not added to the wait-set, so their callbacks never fire. Create all callback groups in the node constructor. ([rclcpp issue #2067](https://github.com/ros2/rclcpp/issues/2067))

**5. Calling `rclpy.spin_once(node)` on a node already attached to an executor.**  
The free function removes and re-attaches the node, severing its existing executor connection. Call `executor.spin_once()` on the executor object itself instead of the free function when mixing manual spin calls with a long-lived executor. ([rclpy issue #1445](https://github.com/ros2/rclpy/issues/1445))

**6. `ReentrantCallbackGroup` without thread-safe shared state.**  
If two instances of the same subscription callback run in parallel and both write to a member variable, the result is a data race. Any shared state touched by a `Reentrant` callback must be protected by `std::mutex` / `threading.Lock()`.

**7. Calling `spin_once` or `spin_until_future_complete` inside a callback on `SingleThreadedExecutor`.**  
This re-enters the executor from inside a callback on the same thread — it deadlocks immediately. Use async patterns (`async_send_request` + done-callback) instead.

---

## Diagnostics

```bash
# Verify executor thread count at runtime
# Each thread should appear separately in htop -H
htop -H  # column: COMM → look for your node executable, one row per executor thread

# Python: check which callback group an entity is assigned to at the REPL
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
node = rclpy.create_node('probe')
timer = node.create_timer(1.0, lambda: None)
print(type(timer.callback_group))   # → MutuallyExclusiveCallbackGroup (default)

# C++: log the executor type in use
RCLCPP_INFO(node->get_logger(), "Spinning with MultiThreadedExecutor, %zu threads",
            executor.get_number_of_threads());
```

For latency profiling under different executors, use the `performance_test` package:
```bash
sudo apt install ros-jazzy-performance-test
ros2 run performance_test perf_test -c rclcpp-single-threaded-executor ...
ros2 run performance_test perf_test -c rclcpp-events-executor ...
```

*(Source: [performance_test — Jazzy docs](https://docs.ros.org/en/ros2_packages/jazzy/api/performance_test/index.html))*

---

## Further reading

- [About Executors — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html) — canonical architecture reference; wait-set model, executor lifecycle, `add_node` semantics
- [Using Callback Groups — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) — complete deadlock-avoidance guide with C++ and Python worked examples
- [ros2/examples — `callback_group.py`, rolling branch (GitHub)](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/callback_group.py) — minimal runnable rclpy executor + callback group demo
- [irobot-ros/events-executor — GitHub](https://github.com/irobot-ros/events-executor) — EventsExecutor design doc, CPU/latency benchmark methodology and raw numbers
- [Class EventsExecutor — rclcpp Rolling API](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/classrclcpp_1_1experimental_1_1executors_1_1EventsExecutor.html) — header path, constructor options, timer thread configuration
- [rclpy.executors module — Jazzy API](https://docs.ros.org/en/ros2_packages/jazzy/api/rclpy/rclpy.executors.html) — full Python executor API: spin, spin_once, spin_until_future_complete signatures
- [rclcpp issue #773 — sync call deadlock](https://github.com/ros2/rclcpp/issues/773) — original deadlock report and analysis for synchronous service calls inside callbacks
- [rclcpp issue #1916 — spin_until_future_complete blocks forever](https://github.com/ros2/rclcpp/issues/1916) — edge case and recommended timeout workaround
- [Improve Executor Performance — ros2/ros2 issue #1395](https://github.com/ros2/ros2/issues/1395) — tracked work that led to StaticSingleThreadedExecutor deprecation and EventsExecutor development

---

*2026-09-26 | ROS2 versions: Humble / Jazzy*
