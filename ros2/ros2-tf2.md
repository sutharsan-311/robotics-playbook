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

On the consumer side, `Buffer` + `TransformListener` subscribe to both topics and maintain the local tree. `buffer.lookup_transform()` / `buffer->lookupTransform()` traverses the tree and returns the composed `TransformStamped`.

---

## REP-105 standard frame hierarchy

[REP-105](https://www.ros.org/reps/rep-0105.html) defines the canonical frame naming convention used by Nav2, MoveIt 2, robot_localization, and every ecosystem package. Deviating from it silently breaks compatibility.

```
earth          (optional — multi-map or GPS-fused systems)
  └── map      (world-fixed; origin = robot start position; can jump on AMCL update)
        └── odom   (world-fixed; odometry-integrated; smooth but drifts over time)
              └── base_link   (rigidly attached to robot chassis, rotational center)
                    ├── base_footprint  (projection of base_link onto the ground plane)
                    ├── imu_link
                    ├── lidar_link
                    └── camera_link
```

**Critical rules:**
- `map → odom` is published by the localization stack (AMCL, Cartographer). It **can jump** when the pose estimate is corrected.
- `odom → base_link` is published by the odometry source (wheel encoders, visual odometry, robot_localization). It is **always continuous and smooth**.
- `base_link → sensor frames` are published by `robot_state_publisher` from URDF + joint states. They are typically static or driven by joint encoders.
- **Only one node may publish each edge.** Two nodes both publishing `odom → base_link` causes oscillating TF and immediate Nav2 failures.

---

## How it works

### Broadcasting a dynamic transform (Python)

The official Jazzy tutorial defines `quaternion_from_euler` inline using `math` and `numpy` — the `tf_transformations` package is not ported to ROS 2 on all distros, so the inline definition is the safe pattern. Code taken from [`learning_tf2_py/turtle_tf2_broadcaster.py`](https://github.com/ros/geometry_tutorials/blob/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py):

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    """Convert roll/pitch/yaw (radians) to a quaternion [x, y, z, w]."""
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs   # x
    q[1] = cj * ss + sj * cc   # y
    q[2] = cj * cs - sj * sc   # z
    q[3] = cj * cc + sj * ss   # w
    return q


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
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
```

**Alternative:** If you have `tf_transformations` installed (`sudo apt install ros-jazzy-tf-transformations`), you can replace the inline function with `from tf_transformations import quaternion_from_euler`. The inline version has no extra dependencies and is what the official Jazzy tutorial ships.

### Listening for a transform (Python)

Code taken from the official Jazzy listener tutorial. Use `TransformException` (not bare `Exception`) to get a meaningful error message that includes which frame is missing:

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self.target_frame = self.declare_parameter(
            'target_frame', 'turtle1').get_parameter_value().string_value

        # Buffer + TransformListener must be stored as members — if created
        # as locals they go out of scope immediately and receive no data.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame = self.target_frame
        to_frame = 'turtle2'
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())   # Time(0) = latest available
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame} to {from_frame}: {ex}')
            return
        # t.transform now holds the composed transform
```

`rclpy.time.Time()` (equivalent to `Time(0)`) requests the **latest available** transform — generally the safest default in a timer-driven loop. Passing `node.get_clock().now()` instead causes `ExtrapolationException` on the first tick before the broadcaster has emitted even one message.

### Broadcasting a dynamic transform (C++)

```cpp
// Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

class FramePublisher : public rclcpp::Node {
public:
    FramePublisher() : Node("turtle_tf2_frame_publisher") {
        turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/" + turtlename_ + "/pose", 10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) { handlePose(msg); });
    }

private:
    void handlePose(const turtlesim::msg::Pose::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtlename_;

        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);   // roll=0, pitch=0, yaw=theta
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    std::string turtlename_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
};
```

In C++, `tf2::Quaternion::setRPY(roll, pitch, yaw)` is the correct API — no external package needed. Never use Euler angles directly in a `TransformStamped`; always convert via `setRPY` or the equivalent.

### Listening for a transform (C++)

```cpp
// Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node {
public:
    FrameListener() : Node("turtle_tf2_frame_listener") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            100ms, [this]() { timerCallback(); });
    }

private:
    void timerCallback() {
        geometry_msgs::msg::TransformStamped t;
        try {
            // tf2::TimePointZero = latest available; 50ms timeout
            t = tf_buffer_->lookupTransform(
                "turtle2", "turtle1",
                tf2::TimePointZero,
                50ms);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }
        // t.transform now holds the composed turtle1 → turtle2 transform
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

`tf2::TimePointZero` is the C++ equivalent of Python's `rclpy.time.Time()` — it requests the latest cached transform, avoiding `ExtrapolationException` on the first few timer ticks.

---

## Transforming data with tf2_geometry_msgs

Looking up a raw `TransformStamped` and then multiplying coordinates manually is error-prone. Use `tf2_geometry_msgs` to transform typed geometry messages directly through the buffer:

```cpp
// CMakeLists.txt: find_package(tf2_geometry_msgs REQUIRED)
// package.xml:    <depend>tf2_geometry_msgs</depend>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

geometry_msgs::msg::PointStamped pt_in, pt_out;
pt_in.header.stamp = this->get_clock()->now();
pt_in.header.frame_id = "base_link";
pt_in.point.x = 1.0;  pt_in.point.y = 0.0;  pt_in.point.z = 0.5;

try {
    // buffer->transform() looks up the transform and applies it in one call
    pt_out = tf_buffer_->transform(pt_in, "map");
} catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
}
// pt_out.point now holds the coordinates in the "map" frame
```

The same `transform()` method works for any `geometry_msgs` stamped type: `PoseStamped`, `Vector3Stamped`, `WrenchStamped`, etc. All specialisations live in `tf2_geometry_msgs/tf2_geometry_msgs.hpp`.

*(Source: [ros2_cookbook — rclcpp/tf2.md, mikeferguson/ros2_cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/tf2.md))*

---

## Time travel

TF2's buffer stores a history window (default 10 s). You can query the transform between two frames **at a past timestamp** — useful for sensor fusion where a point cloud was captured 200 ms ago and you need the robot pose *at that moment*, not now:

```python
# Python: look up transform using the timestamp from an incoming sensor message
try:
    t = self.tf_buffer.lookup_transform(
        'map',
        'base_link',
        sensor_msg.header.stamp,                       # time the data was captured
        timeout=rclpy.duration.Duration(seconds=0.1))
except TransformException as ex:
    self.get_logger().warn(str(ex))
```

```cpp
// C++: look up at a historical timestamp
// Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html
rclcpp::Time when = this->get_clock()->now() - rclcpp::Duration(5, 0);  // 5 s ago
try {
    t = tf_buffer_->lookupTransform("turtle2", "turtle1", when, 50ms);
} catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
}
```

**Advanced fixed-frame time travel** (the six-argument `lookupTransform`): transform the source frame evaluated *at time T* into the target frame evaluated *at time now*, using a fixed anchor frame as the bridge. This is the correct API for a sensor-to-world transform where the sensor moved between capture and processing:

```cpp
// Source: docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp.html
// "Where was carrot1 relative to turtle2 *now*, given carrot1's pose was captured 5 s ago?"
rclcpp::Time past = this->get_clock()->now() - rclcpp::Duration(5, 0);
t = tf_buffer_->lookupTransform(
    "turtle2",                      // target frame, evaluated at *now*
    this->get_clock()->now(),
    "carrot1",                      // source frame, evaluated at *past*
    past,
    "world",                        // fixed frame — invariant across time
    50ms);
```

---

## CLI debugging tools

```bash
# Stream a live transform between two frames
ros2 run tf2_ros tf2_echo world base_link

# Render the full TF tree to a PDF (saved to ./frames_YYYY-MM-DD_HH.MM.SS.pdf)
ros2 run tf2_tools view_frames

# Show per-link timing statistics (average delay, Hz, last update)
ros2 run tf2_ros tf2_monitor base_link odom

# One-shot: print current transform between two frames (wait up to 2 s for it)
ros2 run tf2_ros tf2_echo --wait-for-message-timeout 2.0 map base_link
```

`view_frames` is the first tool to run when debugging TF problems — it generates a PDF of the entire frame graph and makes disconnected subtrees immediately visible. `tf2_monitor` tells you *which broadcaster* is publishing each link and at what rate, which pinpoints a stalled publisher.

---

## Common pitfalls

**1. Calling `quaternion_from_euler` without defining or importing it (Python).**
In the official Jazzy tutorial, `quaternion_from_euler` is defined **inline** using `math` and `numpy`. There is no `from tf2_ros import quaternion_from_euler`. If you copy the function call without the definition, you get a `NameError` at runtime. Either include the inline definition (shown above) or install and import `tf_transformations` (`sudo apt install ros-jazzy-tf-transformations`).

**2. Extrapolation into the future.**
Requesting `lookup_transform` at `node.get_clock().now()` before the broadcaster's first message arrives causes:
`ExtrapolationException: Lookup would require extrapolation into the future`.
Use `rclpy.time.Time()` / `tf2::TimePointZero` (latest) instead of `now()`, or pass a `timeout=Duration(seconds=0.1)` / `50ms` to block until the transform is available.

**3. Not storing `TransformListener` as a member variable.**
`TransformListener` starts background threads and subscriptions when constructed. If created as a local variable it goes out of scope immediately, the subscriptions are torn down, and the `Buffer` never receives any data. Always store it as a member (`self.tf_listener` / `tf_listener_`).

**4. Publishing dynamic transforms with `StaticTransformBroadcaster`.**
`StaticTransformBroadcaster` publishes to `/tf_static` with `TRANSIENT_LOCAL` durability — the last published value is cached and re-delivered to late-joining subscribers, but it is never updated by subsequent `sendTransform()` calls in the same session (the topic is latched). If a frame is dynamic (e.g. an arm link), use `TransformBroadcaster` on `/tf`.

**5. Two nodes publishing the same transform edge.**
TF2 has no arbitration — if both `robot_localization` and a custom node publish `odom → base_link`, the buffer receives interleaved conflicting data. Downstream consumers (Nav2, MoveIt) will see jitter, warnings about TF going backwards in time, or planner crashes. Each transform edge must have exactly one publisher.

**6. Static transforms published on `/tf` instead of `/tf_static`.**
`StaticTransformBroadcaster` uses `TRANSIENT_LOCAL` durability — late-joining nodes automatically receive the cached value. If you accidentally publish static transforms on `/tf` (e.g. by using `TransformBroadcaster` once at startup), late-joining nodes miss them and get `LookupException` until the next rebroadcast.

---

## Further reading

- [TF2 Tutorial Series — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html) — official step-by-step tutorials covering broadcaster, listener, time travel, and frames
- [Writing a TF2 Broadcaster (Python) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html) — full Python broadcaster with inline `quaternion_from_euler` definition
- [Writing a TF2 Listener (C++) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html) — full C++ listener with Buffer setup and lookupTransform
- [Traveling in Time (C++) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp.html) — six-argument lookupTransform and fixed-frame API
- [Debugging TF2 Problems — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html) — systematic walkthrough of tf2_echo, tf2_monitor, and view_frames with real error outputs
- [REP-105 — Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html) — the canonical definition of map, odom, base_link, and earth frames with their transform semantics
- [Setting Up Transformations — Nav2 docs](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html) — REP-105 in practice: which node publishes which edge, robot_localization integration
- [ros2_cookbook — rclcpp/tf2.md (mikeferguson, GitHub)](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/tf2.md) — concise C++ recipes for `buffer.transform()` and `tf2_geometry_msgs`
- [ros/geometry_tutorials (GitHub)](https://github.com/ros/geometry_tutorials) — the canonical C++ and Python broadcaster/listener source referenced throughout the official docs

---

*2026-07-25 | ROS 2 version: Jazzy / Humble*
