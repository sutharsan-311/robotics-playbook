# ROS2 Executors — Single, Multi-threaded, Static

An executor is the scheduling engine that drives callback execution in ROS2 — choosing the wrong one (or misconfiguring callback groups alongside it) is one of the most common sources of subtle deadlocks and unexpected single-threading behaviour in production ROS2 systems.

## What it is

Every `rclcpp` or `rclpy` node needs something to call its callbacks. That something is an **executor**. It owns a thread pool (or a single thread), polls the underlying DDS middleware for ready events, and dispatches callbacks to whichever thread is available.

ROS2 ships three concrete executors:

| Executor | Threads | Wait-set rebuilt each iter? |
|---|---|---|
| `SingleThreadedExecutor` | 1 | Yes (but cheap since Humble) |
| `MultiThreadedExecutor` | N (configurable) | Yes |
| `StaticSingleThreadedExecutor` | 1 | No — **deprecated in Jazzy** |

**`SingleThreadedExecutor`** is the safe default. Callbacks are serialised and run in priority order: timers → subscriptions → services → clients. No data races; no need for mutexes inside callbacks.

**`MultiThreadedExecutor`** creates a configurable thread pool and dispatches callbacks in parallel — but actual parallelism is gated by **callback groups**, not just the thread count.

**`StaticSingleThreadedExecutor`** avoided rebuilding the wait-set on every iteration by caching the entity list at node-add time. Its optimisations have since been merged into the standard executors, and the type is formally deprecated in Jazzy (removed in Rolling). If you still see it in Humble codebases, replace it with `SingleThreadedExecutor`.

## How it works

The critical concept alongside `MultiThreadedExecutor` is **callback groups**. Every entity (subscription, timer, service, action) belongs to exactly one callback group. Two group types exist:

- **`MutuallyExclusiveCallbackGroup`** — at most one callback in the group runs at a time (behaves like a per-group mutex).
- **`ReentrantCallbackGroup`** — the executor may execute callbacks in the group concurrently, including the same callback running against itself.

> **Gotcha:** the node's *default* callback group is `MutuallyExclusive`. If you spin a node in a `MultiThreadedExecutor` without assigning any explicit groups, you still get single-threaded behaviour.

The following example is taken verbatim from the official [`ros2/examples`](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/callback_group.py) repository:

```python
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class DoubleTalker(Node):
    def __init__(self):
        super().__init__('double_talker')
        self.pub = self.create_publisher(String, 'chatter', 10)

        # Only one of these timer callbacks runs at a time,
        # even though the executor has multiple threads.
        self.group = MutuallyExclusiveCallbackGroup()
        self.timer  = self.create_timer(1.0, self.timer_callback, callback_group=self.group)
        self.timer2 = self.create_timer(0.5, self.timer_callback, callback_group=self.group)

def main(args=None):
    with rclpy.init(args=args):
        talker   = DoubleTalker()
        listener = Listener()          # separate node, no shared group

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(talker)
        executor.add_node(listener)
        executor.spin()
```

`listener`'s callbacks run freely in parallel with `talker`'s — but `talker`'s two timers are serialised by the `MutuallyExclusiveCallbackGroup`.

The C++ equivalent uses `create_callback_group` and `SubscriptionOptions`:

```cpp
my_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
rclcpp::SubscriptionOptions opts;
opts.callback_group = my_group;
my_sub = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(), cb, opts);
```
*(Source: [ros2_documentation/Using-callback-groups.rst, humble branch](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Using-callback-groups.rst))*

## Common pitfalls

**1. Deadlock from synchronous service/action calls inside a callback.**
If a timer callback makes a synchronous service call and both the timer and the client belong to the same `MutuallyExclusive` group (or there's only one thread), the response callback can never fire — the thread is stuck waiting for itself. The fix: put the calling callback and the client in *different* callback groups, or switch to async calls. This will give no warning — the node just freezes silently. ([rclcpp issue #773](https://github.com/ros2/rclcpp/issues/773))

**2. `MultiThreadedExecutor` that behaves single-threaded.**
Because the default callback group is `MutuallyExclusive`, simply switching from `SingleThreadedExecutor` to `MultiThreadedExecutor` without assigning explicit callback groups does nothing for throughput. Always assign groups deliberately when you want real parallelism.

**3. Combining multiple nodes in one executor with shared state.**
`executor.add_node()` accepts multiple nodes. Callbacks from different nodes can interleave freely (even under `SingleThreadedExecutor`, their callbacks share one thread and may touch shared data). If nodes share a resource, protect it with a mutex or keep each node in its own executor on its own thread.

## Further reading

- [Executors — ROS 2 Docs (Humble)](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) — canonical concept reference with architecture diagrams
- [Using Callback Groups — ROS 2 Docs (Jazzy)](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html) — deadlock-avoidance guidelines and complete worked examples
- [ros2/examples — callback_group.py (rolling)](https://github.com/ros2/examples/blob/rolling/rclpy/executors/examples_rclpy_executors/callback_group.py) — minimal runnable rclpy executor + callback group demo

---

*2026-08-31 | ROS2 versions: Humble / Jazzy*
