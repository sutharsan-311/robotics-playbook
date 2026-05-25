# ROS2 Nodes — Creating and Managing

A node is the fundamental unit of computation in ROS2 — an isolated process that communicates with peers via topics, services, and actions. Getting node design right determines whether your system is debuggable, composable, and safe to restart.

## What it is

Every ROS2 node is an instance of `rclcpp::Node` (C++) or `rclpy.node.Node` (Python). A node has a unique name within its namespace, owns its communication entities (publishers, subscriptions, timers, services, action servers), and is driven by an **Executor** that dispatches callbacks from a thread pool.

The node name is established at construction time and appears in `ros2 node list`. Two nodes with the same fully-qualified name cause the older instance to be silently killed by the RMW layer — a common source of mysterious restarts in multi-launch systems.

ROS2 also ships `rclcpp_lifecycle::LifecycleNode`, a managed variant that follows a well-defined state machine (unconfigured → inactive → active → finalized). Lifecycle nodes are the right choice for any node that manages hardware resources or requires coordinated startup ordering.

## How it works

Subclass `Node`, create entities in the constructor, and hand the node to an executor:

```python
# Source: https://github.com/ros2/examples/blob/humble/rclpy/topics/minimal_publisher/
#         examples_rclpy_minimal_publisher/publisher_member_function.py

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
```

`rclpy.spin()` wraps a `SingleThreadedExecutor` internally. For concurrent callback execution, construct a `MultiThreadedExecutor` explicitly and assign callbacks to separate `MutuallyExclusiveCallbackGroup` or `ReentrantCallbackGroup` instances.

In C++, the idiom is identical in structure: inherit from `rclcpp::Node`, create entities in the initializer list, then pass a `std::shared_ptr` to `rclcpp::spin()`.

## Common pitfalls

**1. Blocking inside a callback.** `rclpy.spin()` / `rclcpp::spin()` uses a single thread by default. Any `sleep`, synchronous I/O, or nested `spin_until_future_complete()` call inside a callback starves all other callbacks on that executor. Move blocking work to a separate thread or switch to `MultiThreadedExecutor` with appropriate callback groups.

**2. Dropping callback group references.** In rclcpp, a callback group must be stored as a class member. If you assign it to a local variable and let it go out of scope, the executor silently drops the associated callbacks — no error, just silence.

**3. Name and namespace collisions.** Relying on default node names in multi-robot or multi-instance deployments leads to ghost nodes. Always pass `--ros-args -r __node:=<unique_name>` at launch, or parameterize the name in your `LaunchDescription`. Check for collisions with `ros2 node list` before assuming a crash.

## Further reading

- [Writing a simple publisher and subscriber (Python) — ROS 2 Humble docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [About Executors — ROS 2 Humble docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)
- [Managed Nodes (lifecycle) design article — design.ros2.org](https://design.ros2.org/articles/node_lifecycle.html)

---
*2026-05-25 | ROS2 version: Jazzy / Humble*
