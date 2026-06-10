# ROS2 QoS — Quality of Service Policies

ROS 2 exposes DDS Quality of Service settings directly to the developer, giving you precise control over the reliability/latency trade-off for every publisher and subscription — and making that trade-off visible where ROS 1 silently imposed TCP semantics on everything.

## What it is

A QoS **profile** is a bundle of policies passed to `create_publisher` / `create_subscription` that governs how the underlying DDS middleware delivers messages. The six configurable axes are:

| Policy | Options | Default |
|--------|---------|---------|
| **History** | `KEEP_LAST(N)` / `KEEP_ALL` | `KEEP_LAST(10)` |
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | `RELIABLE` |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | `VOLATILE` |
| **Deadline** | duration | unset |
| **Lifespan** | duration | unset |
| **Liveliness** | `AUTOMATIC` / `MANUAL_BY_TOPIC` + lease duration | `AUTOMATIC` |

ROS 2 ships several predefined profiles for the most common patterns:

- **Default** — `KEEP_LAST(10)`, `RELIABLE`, `VOLATILE`
- **Sensor data** — `KEEP_LAST(5)`, `BEST_EFFORT`, `VOLATILE` (use for cameras, LiDAR, IMU)
- **Services** — `KEEP_LAST(10)`, `RELIABLE`, `VOLATILE`
- **Parameters** — `KEEP_LAST(1000)`, `RELIABLE`, `VOLATILE`

## How it works

### Compatibility — "Request vs Offered"

A publisher/subscription connection is made only when their QoS profiles are **compatible**. The model is asymmetric: the subscription *requests* a minimum acceptable quality; the publisher *offers* a maximum it can provide. The connection forms only if the offered quality satisfies the request.

Practical compatibility table for the two most-tripped policies:

| Publisher | Subscriber | Connect? |
|-----------|-----------|----------|
| `RELIABLE` | `RELIABLE` | ✅ |
| `RELIABLE` | `BEST_EFFORT` | ✅ |
| `BEST_EFFORT` | `RELIABLE` | ❌ |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | ✅ (late-joiner gets cached msgs) |
| `TRANSIENT_LOCAL` | `VOLATILE` | ✅ (no late-join cache) |
| `VOLATILE` | `TRANSIENT_LOCAL` | ❌ |

### Code — rclpy

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Custom profile: reliable + transient local — good for /map or /costmap
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)

# Use the predefined sensor data profile for high-rate streams
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
```

*(Source: [ros2/rclpy qos.py](https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/qos.py), [ros2/demos lifespan.py](https://github.com/ros2/demos/blob/master/quality_of_service_demo/rclpy/quality_of_service_demo_py/lifespan.py))*

### Code — rclcpp

```cpp
// Fluent builder API
auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", qos, callback);

// Predefined profile for sensor streams
auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS(), callback);
```

*(Source: [rclcpp QoS class — Rolling API](https://docs.ros.org/en/ros2_packages/rolling/api/rclcpp/generated/classrclcpp_1_1QoS.html), [rclcpp::SensorDataQoS](https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1SensorDataQoS.html))*

## Common pitfalls

**1. Silent connection failure on QoS mismatch**
This is the hardest one to debug: the topic appears in `ros2 topic list`, both nodes are alive, but no messages arrive. The root cause is a DDS-level incompatibility — it refuses the connection without raising an error in application code. Diagnose it immediately with:
```bash
ros2 topic info /your_topic --verbose
```
Look for `"Incompatible QoS"` events or mismatched reliability/durability in the publisher vs subscriber lines.

**2. Camera/LiDAR drivers publish `BEST_EFFORT` — your subscriber defaults to `RELIABLE`**
Most hardware driver nodes use `BEST_EFFORT` on sensor topics to trade reliability for low latency. Any subscriber you write with the default QoS (`RELIABLE`) will silently fail to connect. Always check driver documentation and match, or use `qos_profile_sensor_data`.

**3. Transient local requires both sides to opt in**
`TRANSIENT_LOCAL` durability gives you "latched topic" semantics — a late-joining subscriber receives the last published message on connect. But if the subscriber is `VOLATILE` the connection forms, it just won't receive the cached data. Worse: if the subscriber is `TRANSIENT_LOCAL` but the publisher is `VOLATILE`, the connection is refused entirely (see compatibility table). The `/map` and `/robot_description` topics are the most common victims of this confusion.

## Further reading

- [About Quality of Service Settings — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 QoS design article — design.ros2.org](https://design.ros2.org/articles/qos.html)
- [Using QoS settings for lossy networks — ROS 2 Tutorial (Humble)](https://docs.ros.org/en/humble/Tutorials/Demos/Quality-of-Service.html)

---
*2026-06-10 | ROS2 version: Jazzy / Humble*
