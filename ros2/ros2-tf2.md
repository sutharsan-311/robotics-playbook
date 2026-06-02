# ROS2 TF2 — Coordinate Transformations

TF2 is the transform library at the core of every non-trivial ROS 2 system: it tracks how every coordinate frame in your robot relates to every other frame, across time, so that a depth image, a lidar scan, and a joint encoder can all be reasoned about in the same space without writing a line of geometry yourself.

---

## What it is

TF2 maintains a **directed tree of coordinate frames**, each edge being a time-stamped rigid transform (translation + quaternion rotation). Nodes broadcast transforms onto `/tf` and `/tf_static`; any node can query the tree at any past time within the buffer window (default 10 s) and get back the composed transform between any two frames.

Two transport roles:

| Role | Class | Topic | Use when |
|---|---|---|---|
| Dynamic broadcaster | `TransformBroadcaster` | `/tf` | Frame changes over time (robot links, moving sensors) |
| Static broadcaster | `StaticTransformBroadcaster` | `/tf_static` | Fixed mounts (camera-to-base, lidar-to-base) |

On the consumer side, `Buffer` + `TransformListener` subscribe to both topics and maintain the local tree. `buffer.lookup_transform()` traverses the tree and returns the composed `TransformStamped`.

---

## How it works

### Broadcasting a dynamic transform

The example below is taken directly from the official `geometry_tutorials` repository (Apache 2.0). It publishes the world → turtle frame each time the turtle's pose updates:

```python
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')
        self.turtlename = self.declare_parameter(
            'turtlename', 'turtle').get_parameter_value().string_value

        # One broadcaster per node is sufficient
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Pose, f'/{self.turtlename}/pose', self.handle_turtle_pose, 1)

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # Turtlesim is 2-D; rotation is only around Z
        q = quaternion_from_euler(0, 0, msg.theta)  # returns [x, y, z, w]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
```

### Listening for a transform

```python
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# In a timer or callback:
try:
    t = self.tf_buffer.lookup_transform(
        'turtle2',          # target frame
        'turtle1',          # source frame
        rclpy.time.Time())  # latest available
except Exception as e:
    self.get_logger().warn(str(e))
```

`rclpy.time.Time()` (equivalent to `Time(0)`) requests the **latest available** transform — generally the safest default in a timer-driven loop.

### CLI debugging tools

```bash
# Stream a live transform between two frames
ros2 run tf2_ros tf2_echo world base_link

# Render the full TF tree to a PDF
ros2 run tf2_tools view_frames

# Show per-link timing statistics (average delay, Hz)
ros2 run tf2_ros tf2_monitor base_link odom
```

---

## Common pitfalls

**1. Extrapolation into the future.**  
Requesting `lookup_transform` at `node.get_clock().now()` before the broadcaster's first message arrives causes:  
`ExtrapolationException: Lookup would require extrapolation into the future`.  
Use `rclpy.time.Time()` (latest) instead of `now()`, or pass a `timeout=Duration(seconds=0.1)` to block until the transform is available.

**2. Missing frames silently breaking the tree.**  
If any link in the chain from source → target is absent, the lookup raises `LookupException: ... does not exist`. Always wrap `lookup_transform` in a `try/except` and log the frame name — the error message tells you exactly which frame is missing. Run `view_frames` to visualise gaps.

**3. Publishing `/tf` at the wrong rate or in the wrong callback.**  
Emitting a transform once at startup inside `__init__` is only correct for static frames (use `StaticTransformBroadcaster`). Dynamic frames must be re-published every time the underlying state changes, typically inside a subscription callback or a high-frequency timer. A stale TF causes downstream nodes (Nav2, MoveIt) to refuse transforms silently.

---

## Further reading

- [TF2 Tutorial Series — ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) — official step-by-step tutorials covering broadcaster, listener, time travel, and frames
- [Debugging TF2 Problems — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html) — systematic walkthrough of tf2_echo, tf2_monitor, and view_frames with real error outputs
- [ros/geometry_tutorials (GitHub)](https://github.com/ros/geometry_tutorials) — the canonical C++ and Python example code referenced throughout the official docs

---

*2026-06-02 | ROS 2 version: Jazzy / Humble*
