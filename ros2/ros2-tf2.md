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

## How it works

### Broadcasting a dynamic transform (Python)

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

### Listening for a transform (Python)

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
        q.setRPY(0, 0, msg->theta);
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
            // 50ms timeout — blocks until the transform is available or throws
            t = tf_buffer_->lookupTransform(
                "turtle2", "turtle1",
                tf2::TimePointZero,    // latest available transform
                50ms);                 // wait up to 50 ms
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }
        // t.transform now holds the composed turtle1→turtle2 transform
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

`tf2::TimePointZero` is the C++ equivalent of Python's `rclpy.time.Time()` — it requests the latest cached transform, avoiding ExtrapolationException on the first few timer ticks.

---

## Transforming data with tf2_geometry_msgs

Looking up a raw `TransformStamped` and then multiplying coordinates manually is error-prone. Use `tf2_geometry_msgs` to transform typed geometry messages directly through the buffer:

```cpp
// CMakeLists.txt: find_package(tf2_geometry_msgs REQUIRED)
// package.xml:    <depend>tf2_geometry_msgs</depend>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Stamp the point in its source frame:
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
        sensor_msg.header.stamp,            # time the data was captured
        timeout=rclpy.duration.Duration(seconds=0.1))
except Exception as e:
    self.get_logger().warn(str(e))
```

```cpp
// C++: look up the transform at a historical timestamp (Jazzy docs)
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
// "Where was carrot1 relative to turtle2 *now*, given that carrot1's pose was captured 5 s ago?"
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

# One-shot: print current transform between two frames
ros2 run tf2_ros tf2_echo --wait-for-message-timeout 2.0 map base_link
```

---

## Common pitfalls

**1. Extrapolation into the future.**  
Requesting `lookup_transform` at `node.get_clock().now()` before the broadcaster's first message arrives causes:  
`ExtrapolationException: Lookup would require extrapolation into the future`.  
Use `rclpy.time.Time()` / `tf2::TimePointZero` (latest) instead of `now()`, or pass a `timeout=Duration(seconds=0.1)` / `50ms` to block until the transform is available.

**2. Missing frames silently breaking the tree.**  
If any link in the chain from source → target is absent, the lookup raises `LookupException: ... does not exist`. Always wrap `lookup_transform` in a `try/except` / `try/catch` and log the frame name — the error message tells you exactly which frame is missing. Run `view_frames` to visualise gaps.

**3. Publishing `/tf` at the wrong rate or in the wrong callback.**  
Emitting a transform once at startup inside `__init__` / the constructor is only correct for static frames (use `StaticTransformBroadcaster`). Dynamic frames must be re-published every time the underlying state changes, typically inside a subscription callback or a high-frequency timer. A stale TF causes downstream nodes (Nav2, MoveIt) to refuse transforms silently.

**4. Not storing the `TransformListener` as a member variable.**  
`TransformListener` starts background threads and subscriptions when constructed. If created as a local variable it goes out of scope immediately, the subscriptions are torn down, and the `Buffer` never receives any data. Always store it as a member (`self.tf_listener` / `tf_listener_`).

**5. Static transforms published repeatedly on `/tf`.**  
`StaticTransformBroadcaster` publishes to `/tf_static`, which uses TRANSIENT_LOCAL durability — late-joining nodes automatically receive the cached value. If you accidentally publish static transforms on `/tf`, late-joining nodes miss them and get `LookupException` until the next rebroadcast.

---

## Further reading

- [TF2 Tutorial Series — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html) — official step-by-step tutorials covering broadcaster, listener, time travel, and frames
- [Writing a TF2 Listener (C++) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html) — full C++ listener with Buffer setup and lookupTransform
- [Traveling in Time (C++) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp.html) — six-argument lookupTransform and fixed-frame API
- [Debugging TF2 Problems — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html) — systematic walkthrough of tf2_echo, tf2_monitor, and view_frames with real error outputs
- [ros2_cookbook — rclcpp/tf2.md (mikeferguson, GitHub)](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/tf2.md) — concise C++ recipes for buffer.transform() and tf2_geometry_msgs
- [ros/geometry_tutorials (GitHub)](https://github.com/ros/geometry_tutorials) — the canonical C++ and Python broadcaster/listener source referenced throughout the official docs

---

*2026-06-20 | ROS 2 version: Jazzy / Humble*
