# ROS2 Architecture Overview

**ROS2 is a ground-up redesign of the Robot Operating System built on industry-standard DDS middleware, eliminating the single-point-of-failure ROS Master and unlocking real-time, multi-robot, and security-critical deployments.**

---

## What it is

ROS2 is a modular, distributed robotics middleware framework. Unlike ROS1, it has no central broker (no `roscore`). Every node is a peer that discovers others autonomously via the underlying DDS transport. The stack layers from application down to wire look like this:

```
Your Node (rclpy / rclcpp)
        │
   rcl  (C API — shared core)
        │
   rmw  (ROS Middleware Interface — DDS abstraction)
        │
   DDS implementation (Fast DDS / Cyclone DDS / Connext / GurumDDS)
        │
   UDP/IP (RTPS wire protocol)
```

**rcl** is the thin C library that implements ROS2 concepts (nodes, topics, services, timers). **rclcpp** and **rclpy** are idiomatic wrappers on top. **rmw** decouples the application from the specific DDS vendor — you can swap implementations at runtime via `RMW_IMPLEMENTATION` without recompiling application code.

The four fundamental communication primitives:

| Primitive | Pattern | Use when |
|-----------|---------|----------|
| **Topic** | Pub/Sub (async, fire-and-forget) | Sensor streams, odometry |
| **Service** | Request/Response (synchronous) | One-shot queries, mode changes |
| **Action** | Goal/Feedback/Result (async, preemptible) | Long-running tasks (navigation, arm motion) |
| **Parameter** | Key-value store per node | Runtime configuration |

---

## How it works

Node discovery is handled entirely by DDS Simple Discovery Protocol (SDP). When a node starts, it broadcasts its presence on the UDP multicast channel for its **ROS_DOMAIN_ID** (0–101). Other nodes on the same domain respond; DDS negotiates the transport and QoS contracts, then opens a direct peer-to-peer data path — no intermediary involved.

The following minimal publisher is taken directly from the [ROS2 Humble official tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Key points in the code:
- `create_publisher(String, 'topic', 10)` — the `10` is the **QoS history depth** (how many messages to buffer). This is *not* a reliability setting.
- `create_timer()` drives the spin loop; a single `rclpy.spin()` handles all callbacks via a single-threaded executor by default.
- `destroy_node()` + `rclpy.shutdown()` cleanly releases DDS resources — skipping these causes `rcl` to log warnings about leaked handles.

For real-time or multi-callback scenarios, swap the default executor:

```python
from rclpy.executors import MultiThreadedExecutor
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

---

## Common pitfalls

**1. QoS policy mismatches cause silent communication failures.**
DDS negotiates QoS at connection time. A publisher using `BEST_EFFORT` reliability paired with a subscriber requiring `RELIABLE` will never exchange data — and ROS2 will not raise an exception. Use `ros2 topic info -v <topic>` to inspect the negotiated QoS on both sides. Camera drivers frequently default to `BEST_EFFORT`; RViz2 defaults to `RELIABLE`.

**2. ROS_DOMAIN_ID isolation is easy to forget.**
Nodes on different `ROS_DOMAIN_ID` values are completely invisible to each other. On a shared lab network with multiple developers, collisions are common. Set it explicitly in `.bashrc` or your launch environment; never rely on the default (0).

**3. Blocking calls inside callbacks starve the executor.**
A `time.sleep()` or a synchronous service call inside a timer callback blocks the single-threaded executor, preventing all other callbacks from firing. Move blocking work to a thread or use `MultiThreadedExecutor` with `ReentrantCallbackGroup`.

**4. Lifecycle nodes add complexity — use them intentionally.**
`LifecycleNode` gives you configure/activate/deactivate/cleanup states for hardware drivers. The overhead is worthwhile for drivers but over-engineering for pure data-processing nodes.

**5. Not pinning the RMW implementation.**
The default DDS vendor differs between distributions and installation methods. Pin `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (or your chosen vendor) in your launch scripts to guarantee consistent behavior across machines.

---

## Further reading

- [ROS 2 Concepts — Official Humble Docs](https://docs.ros.org/en/humble/Concepts.html)
- [About Different Middleware Vendors — ROS 2 Rolling Docs](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Different-Middleware-Vendors.html)
- [ROS 2 Architecture Patterns That Scale (Topics, Services, Actions, Lifecycle)](https://thomasthelliez.com/blog/ros-2-architecture-patterns-that-scale/)

---

*2026-05-23 | ROS2 Jazzy/Humble*
