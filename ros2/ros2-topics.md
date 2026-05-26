# ROS2 Topics — Publishers and Subscribers

Topics are the primary communication primitive in ROS2: named, typed, many-to-many message buses that fully decouple producers from consumers. A publisher doesn't need to know who's listening; a subscriber doesn't need to know who's publishing — this decoupling is what makes ROS2 graphs introspectable, recomposable, and safe to evolve independently.

## What it is

A topic is a named channel — e.g. `/scan`, `/cmd_vel`, `/joint_states` — over which nodes exchange typed messages. Any number of publishers and subscribers can coexist on a single topic simultaneously. The underlying transport is DDS (Data Distribution Service), which handles peer discovery, serialization, and delivery transparently across processes and machines.

Topic names follow a hierarchical namespace convention. A node in namespace `my_robot` publishing to the relative name `cmd_vel` resolves to `/my_robot/cmd_vel`. An absolute name (leading `/`) bypasses namespace prepending entirely. The tilde prefix (`~/odom`) expands to the node's fully-qualified name — useful for self-contained nodes that must not collide in multi-robot deployments.

Topics are strongly typed: publisher and subscriber must agree on the message type (e.g. `std_msgs/msg/String`, `geometry_msgs/msg/Twist`, or a custom `.msg` definition). In Python the mismatch surfaces at runtime; in C++ it fails at compile time.

## How it works

The publisher and subscriber APIs in `rclpy` are symmetrical. Below is the canonical example verbatim from the official [`ros2/examples`](https://github.com/ros2/examples/tree/rolling/rclpy/topics) repository (rolling branch):

**Publisher** — `publisher_member_function.py`:
```python
# Source: github.com/ros2/examples — rclpy/topics/minimal_publisher/
#         examples_rclpy_minimal_publisher/publisher_member_function.py

import rclpy
from rclpy.executors import ExternalShutdownException
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
    try:
        with rclpy.init(args=args):
            minimal_publisher = MinimalPublisher()
            rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
```

**Subscriber** — `subscriber_member_function.py`:
```python
# Source: github.com/ros2/examples — rclpy/topics/minimal_subscriber/
#         examples_rclpy_minimal_subscriber/subscriber_member_function.py

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

The integer `10` passed to both calls is the **queue depth** — shorthand for `QoSProfile(history=KeepLast(10))`. It controls how many undelivered messages DDS buffers before dropping. For high-rate sensor streams, tune this to match your worst-case processing lag; for control topics, keep it small (1–5) to avoid stale commands.

Inspect topics at runtime with the CLI:
```
ros2 topic list            # all active topics in the graph
ros2 topic info -v /topic  # publishers, subscribers, and full QoS profiles
ros2 topic echo /topic     # live message stream
ros2 topic hz /topic       # measured publish rate
```

## Common pitfalls

**1. Silent QoS incompatibility.** DDS refuses to connect a publisher and subscriber with incompatible QoS profiles — no error, no warning, just silence. The most common case is **durability**: the default is `VOLATILE`, but system topics like `/robot_description` use `TRANSIENT_LOCAL` (latching). A subscriber that joins late with the default `VOLATILE` durability receives nothing. Both sides must explicitly set `transient_local` durability for latching to work. Always diagnose with `ros2 topic info -v /your_topic` before assuming a node is broken.

**2. Blocking inside a subscriber callback.** `rclpy.spin()` uses a `SingleThreadedExecutor` by default. Any `time.sleep()`, blocking I/O, or synchronous service call inside `listener_callback` stalls all other callbacks on that node executor. Move heavy processing to a thread pool, or construct a `MultiThreadedExecutor` with a `ReentrantCallbackGroup`.

**3. Namespace surprises after remapping.** Forgetting that relative topic names are prepended with the node's namespace causes topics to appear under unexpected paths in multi-robot or namespaced-launch deployments. Always run `ros2 topic list` after launching a new configuration and prefer fully-qualified names (`/cmd_vel`) in cross-package subscribers where accidental remapping would be dangerous.

## Further reading

- [About Topics — ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Topics.html)
- [Quality of Service settings — ROS 2 Rolling documentation](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ros2/examples — minimal publisher & subscriber source (GitHub)](https://github.com/ros2/examples/tree/rolling/rclpy/topics)

---
*2026-05-26 | ROS2 version: Jazzy / Humble*
