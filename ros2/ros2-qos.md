# ROS2 QoS — Quality of Service Policies

ROS 2 exposes DDS Quality of Service settings directly to the developer, giving you precise control over the reliability/latency trade-off for every publisher and subscription — and making that trade-off visible where ROS 1 silently imposed TCP semantics on everything.

---

## What it is

A QoS **profile** is a bundle of policies passed to `create_publisher` / `create_subscription` that governs how the underlying DDS middleware delivers messages. The six configurable axes are:

| Policy | Options | Default |
|--------|---------|---------|
| **History** | `KEEP_LAST(N)` / `KEEP_ALL` | `KEEP_LAST(10)` |
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | `RELIABLE` |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | `VOLATILE` |
| **Deadline** | `Duration` — max gap between consecutive messages | unset (infinite) |
| **Lifespan** | `Duration` — max age of a message before it is dropped | unset (infinite) |
| **Liveliness** | `AUTOMATIC` / `MANUAL_BY_TOPIC` + lease duration | `AUTOMATIC` |

ROS 2 ships several predefined profiles for the most common patterns:

- **Default** — `KEEP_LAST(10)`, `RELIABLE`, `VOLATILE`
- **Sensor data** — `KEEP_LAST(5)`, `BEST_EFFORT`, `VOLATILE` (cameras, LiDAR, IMU)
- **Services** — `KEEP_LAST(10)`, `RELIABLE`, `VOLATILE`
- **Parameters** — `KEEP_LAST(1000)`, `RELIABLE`, `VOLATILE`

---

## Compatibility — "Request vs Offered"

A publisher/subscription connection forms only when their QoS profiles are **compatible**. The model is asymmetric: the subscription *requests* a minimum acceptable quality; the publisher *offers* a maximum it can provide. The connection forms only if the offered quality satisfies the request.

Practical compatibility table for the most-tripped policies:

| Publisher | Subscriber | Connect? |
|-----------|-----------|----------|
| `RELIABLE` | `RELIABLE` | ✅ |
| `RELIABLE` | `BEST_EFFORT` | ✅ |
| `BEST_EFFORT` | `RELIABLE` | ❌ — silent failure |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | ✅ (late-joiner gets cached msgs) |
| `TRANSIENT_LOCAL` | `VOLATILE` | ✅ (no late-join cache) |
| `VOLATILE` | `TRANSIENT_LOCAL` | ❌ — silent failure |

For Deadline and Liveliness, compatibility follows the same offered ≥ requested model: a subscriber requesting a 500 ms deadline will not connect to a publisher offering a 1 s (looser) deadline.

---

## Reliability and Durability — rclpy

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Custom profile: reliable + transient local — for /map or /costmap
qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)

# Predefined sensor data profile for high-rate streams
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile_sensor_data)
```

*(Source: [ros2/rclpy — qos.py](https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/qos.py))*

## Reliability and Durability — rclcpp

```cpp
// Fluent builder API
auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", qos, callback);

// Predefined sensor data profile
auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS(), callback);
```

*(Source: [rclcpp QoS class — Jazzy API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1QoS.html))*

---

## Deadline

**Deadline** declares the maximum period between consecutive messages on a topic. Both publisher and subscription specify a deadline period; the connection forms only if the publisher's offered deadline ≤ the subscriber's requested deadline.

When the deadline is missed, the middleware fires an **event callback** on both sides — the only reliable signal that a node has stalled, hardware has dropped out, or a compute pipeline has over-run its budget.

### rclpy — Deadline with event callbacks

Adapted from [`quality_of_service_demo/rclpy/quality_of_service_demo_py/deadline.py`](https://github.com/ros2/demos/blob/jazzy/quality_of_service_demo/rclpy/quality_of_service_demo_py/deadline.py) (Apache 2.0, ros2/demos):

```python
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos import QoSProfile

# 500 ms deadline — publisher must publish at ≥ 2 Hz, or the event fires
deadline = Duration(seconds=0.5)
qos_profile = QoSProfile(depth=10, deadline=deadline)

# Subscription event: fires when a message was not received within the deadline
def sub_deadline_event(event):
    count = event.total_count
    delta = event.total_count_change
    get_logger('listener').info(
        f'Requested deadline missed - total {count} delta {delta}')

subscription_callbacks = SubscriptionEventCallbacks(deadline=sub_deadline_event)

# Publisher event: fires when the publisher itself missed the deadline
def pub_deadline_event(event):
    count = event.total_count
    delta = event.total_count_change
    get_logger('talker').info(
        f'Offered deadline missed - total {count} delta {delta}')

publisher_callbacks = PublisherEventCallbacks(deadline=pub_deadline_event)

# Pass event_callbacks when creating publisher/subscription
publisher = node.create_publisher(String, 'my_topic', qos_profile,
                                  event_callbacks=publisher_callbacks)
subscription = node.create_subscription(String, 'my_topic', callback, qos_profile,
                                        event_callbacks=subscription_callbacks)
```

### rclcpp — Deadline with event callbacks

Adapted from [`quality_of_service_demo/rclcpp/src/deadline.cpp`](https://github.com/ros2/demos/blob/jazzy/quality_of_service_demo/rclcpp/src/deadline.cpp) (Apache 2.0, ros2/demos):

```cpp
#include "rclcpp/rclcpp.hpp"
#include <chrono>
using namespace std::chrono_literals;

rclcpp::QoS qos_profile(10);
qos_profile.deadline(500ms);   // 500 ms deadline

// Publisher-side: fired when the publisher missed its own deadline
auto talker = std::make_shared<MyTalkerNode>(qos_profile, "my_topic");
talker->get_options().event_callbacks.deadline_callback =
    [node = talker.get()](rclcpp::QOSDeadlineOfferedInfo & event) -> void {
        RCLCPP_WARN(node->get_logger(),
            "Offered deadline missed - total %d delta %d",
            event.total_count, event.total_count_change);
    };

// Subscription-side: fired when no message arrived within the deadline
auto listener = std::make_shared<MyListenerNode>(qos_profile, "my_topic");
listener->get_options().event_callbacks.deadline_callback =
    [node = listener.get()](rclcpp::QOSDeadlineRequestedInfo & event) -> void {
        RCLCPP_WARN(node->get_logger(),
            "Requested deadline missed - total %d delta %d",
            event.total_count, event.total_count_change);
    };

talker->initialize();
listener->initialize();
```

**Practical use:** set a deadline tighter than your control loop period on `/cmd_vel` or `/joint_states`. The deadline callback becomes your watchdog — if the publisher stalls (computation overrun, crashed node, dropped network packet), the subscription knows immediately without polling.

---

## Lifespan

**Lifespan** sets a maximum age for messages stored in the publisher's history queue. A message that has been sitting in the queue longer than the lifespan is dropped before delivery — it is never seen by a late-joining subscriber.

Lifespan is only meaningful when combined with `TRANSIENT_LOCAL` durability (the late-joiner cache). Without it, messages are never cached and the setting has no effect.

### rclpy — Lifespan

Adapted from [`quality_of_service_demo/rclpy/quality_of_service_demo_py/lifespan.py`](https://github.com/ros2/demos/blob/jazzy/quality_of_service_demo/rclpy/quality_of_service_demo_py/lifespan.py) (Apache 2.0, ros2/demos):

```python
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

lifespan = Duration(seconds=1.0)

qos_profile = QoSProfile(
    depth=10,
    # RELIABLE required so the late-joiner cache is used
    reliability=QoSReliabilityPolicy.RELIABLE,
    # TRANSIENT_LOCAL: publisher keeps messages for late-joining subscribers
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    lifespan=lifespan)

publisher = node.create_publisher(String, 'my_topic', qos_profile)
```

A subscriber that joins 0.8 s after the publisher emitted a message with `lifespan=1.0 s` will still receive it. A subscriber joining at 1.2 s will not — the message has expired and been discarded from the cache.

---

## Liveliness

**Liveliness** asserts that a publisher is still alive. Subscribers receive an event callback when the number of live publishers on a topic changes — the primary mechanism for detecting a crashed or silent publisher.

Two policies:

| Policy | Who asserts liveliness | How |
|--------|------------------------|-----|
| `AUTOMATIC` | The RMW layer — any DDS activity (publish, callback) resets the lease | Automatic, no user code needed |
| `MANUAL_BY_TOPIC` | The user — must call `publisher.assert_liveliness()` within the lease duration | Explicit; survives a node that is alive but not publishing |

### rclpy — Liveliness with event callbacks

Adapted from [`quality_of_service_demo/rclpy/quality_of_service_demo_py/liveliness.py`](https://github.com/ros2/demos/blob/jazzy/quality_of_service_demo/rclpy/quality_of_service_demo_py/liveliness.py) (Apache 2.0, ros2/demos):

```python
from rclpy.duration import Duration
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos import QoSLivelinessPolicy, QoSProfile

liveliness_lease_duration = Duration(seconds=1.0)

# AUTOMATIC: RMW resets the lease whenever the node does any DDS activity
qos_profile = QoSProfile(
    depth=10,
    liveliness=QoSLivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=liveliness_lease_duration)

# Fired on the subscription side when the count of live publishers changes
def sub_liveliness_event(event):
    get_logger('listener').info('Liveliness changed event:')
    get_logger('listener').info(f'  alive_count: {event.alive_count}')
    get_logger('listener').info(f'  not_alive_count: {event.not_alive_count}')
    get_logger('listener').info(f'  alive_count_change: {event.alive_count_change}')
    get_logger('listener').info(f'  not_alive_count_change: {event.not_alive_count_change}')

subscription_callbacks = SubscriptionEventCallbacks(liveliness=sub_liveliness_event)
subscription = node.create_subscription(String, 'my_topic', callback, qos_profile,
                                        event_callbacks=subscription_callbacks)
```

For `MANUAL_BY_TOPIC`, call `publisher.assert_liveliness()` on a timer whose period is shorter than the lease duration:

```python
# Source: quality_of_service_demo_py/common_nodes.py — Talker class (ros2/demos, Apache 2.0)

qos_profile = QoSProfile(
    depth=10,
    liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
    liveliness_lease_duration=Duration(seconds=1.0))

publisher = node.create_publisher(String, 'my_topic', qos_profile)

# Must call assert_liveliness() faster than the lease duration, even when not publishing
assert_timer = node.create_timer(0.5, publisher.assert_liveliness)
```

If `assert_liveliness()` is not called within the lease duration, the subscription's liveliness callback fires with `not_alive_count` incremented — the DDS-level signal that the publisher has gone silent.

### rclcpp — Liveliness

Adapted from [`quality_of_service_demo/rclcpp/src/liveliness.cpp`](https://github.com/ros2/demos/blob/jazzy/quality_of_service_demo/rclcpp/src/liveliness.cpp) (Apache 2.0, ros2/demos):

```cpp
#include "rclcpp/rclcpp.hpp"
#include <chrono>
using namespace std::chrono_literals;

rclcpp::QoS qos_profile(10);
qos_profile
    .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
    .liveliness_lease_duration(1000ms);

auto listener = std::make_shared<MyListenerNode>(qos_profile, "my_topic");
listener->get_options().event_callbacks.liveliness_callback =
    [listener](rclcpp::QOSLivelinessChangedInfo & event) {
        RCLCPP_INFO(listener->get_logger(), "Liveliness changed event:");
        RCLCPP_INFO(listener->get_logger(), "  alive_count: %d", event.alive_count);
        RCLCPP_INFO(listener->get_logger(), "  not_alive_count: %d", event.not_alive_count);
    };
```

---

## Runtime QoS overrides via parameters

`rclcpp` exposes a consistent way to override QoS settings per-node through ROS 2 parameters, without recompiling:

```cpp
// Source: mikeferguson/ros2_cookbook — pages/qos.md
rclcpp::PublisherOptions pub_options;
pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
node->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(10), pub_options);
```

Then in a parameter YAML:
```yaml
my_node:
  ros__parameters:
    qos_overrides:
      /fully/resolved/topic:
        publisher:
          reliability: reliable
          depth: 100
          history: keep_last
```

The same pattern works for subscriptions — use `rclcpp::SubscriptionOptions` and replace `publisher:` with `subscription:` in the YAML. `with_default_policies()` allows overriding `history`, `depth`, and `reliability`. To expose additional axes, pass an initializer list to `QosOverridingOptions`:

```cpp
pub_options.qos_overriding_options = rclcpp::QosOverridingOptions(
    {rclcpp::QosPolicyKind::Reliability, rclcpp::QosPolicyKind::Durability});
```

*(Source: [mikeferguson/ros2_cookbook — qos.md](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/qos.md))*

---

## Diagnosing QoS problems

```bash
# Show per-publisher/subscription QoS — the first command to run on a silent topic
ros2 topic info /your_topic --verbose

# Check for IncompatibleQoS events (printed by the RMW layer when a connection is refused)
ros2 topic info /your_topic --verbose | grep -A2 "Incompatible"

# Show what QoS is currently offered/requested on a Jazzy system
ros2 topic echo --qos-reliability best_effort /your_topic
```

---

## Common pitfalls

**1. Silent connection failure on QoS mismatch.**
The topic appears in `ros2 topic list`, both nodes are alive, but no messages arrive. The root cause is a DDS-level incompatibility — refused without an error in application code. Diagnose immediately with `ros2 topic info /your_topic --verbose` and look for the `Incompatible QoS` events or mismatched reliability/durability lines.

**2. Camera/LiDAR drivers publish `BEST_EFFORT` — your subscriber defaults to `RELIABLE`.**
Most hardware driver nodes (and the `image_transport` stack) use `BEST_EFFORT` on sensor topics for low latency. Any subscriber you write with the default QoS (`RELIABLE`) silently fails to connect. Always check driver documentation and match, or use `qos_profile_sensor_data`.

**3. `TRANSIENT_LOCAL` requires both sides to opt in.**
`TRANSIENT_LOCAL` gives you latched-topic semantics — a late-joining subscriber receives cached messages on connect. If the subscriber is `VOLATILE`, the connection forms but the cache is never delivered. Worse: if the subscriber is `TRANSIENT_LOCAL` but the publisher is `VOLATILE`, the connection is refused entirely. The `/map` and `/robot_description` topics are the most common victims.

**4. Deadline and Liveliness events never fire without matching policy on both sides.**
Setting a deadline on only the publisher or only the subscription does not produce events — the DDS connection forms (deadline compatibility is only checked if both sides specify a finite duration), but no events are generated. Both publisher and subscription must specify a finite deadline with the same or compatible duration for event callbacks to trigger reliably.

**5. Lifespan without `TRANSIENT_LOCAL` has no observable effect.**
The lifespan policy governs the age of messages sitting in the publisher's late-joiner cache. If the publisher uses `VOLATILE` durability (the default), there is no cache — messages are delivered immediately or not at all, and the lifespan timer never has anything to expire. Lifespan is only meaningful with `TRANSIENT_LOCAL` + `RELIABLE`.

---

## Further reading

- [About Quality of Service Settings — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html) — full policy reference, compatibility table, DDS mapping
- [ROS QoS — Deadline, Liveliness, and Lifespan (design.ros2.org)](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html) — design rationale and DDS semantics behind the three time-based policies
- [Quality of Service demo — ROS 2 Jazzy Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Demos/Quality-of-Service.html) — interactive CLI demo for sensor data QoS on a lossy network
- [ros2/demos — quality_of_service_demo (GitHub)](https://github.com/ros2/demos/tree/jazzy/quality_of_service_demo) — full rclpy and rclcpp source for deadline, lifespan, and liveliness demos used in this article
- [REP-2003 — Recommended QoS settings for standard topics](https://ros.org/reps/rep-2003.html) — canonical QoS profiles for maps, sensor drivers, and service interfaces
- [mikeferguson/ros2_cookbook — qos.md (GitHub)](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/qos.md) — runtime QoS overrides via parameters, SensorDataQoS recommendations, and the "magic number" timeout explanation

---

*2026-07-18 | ROS2 version: Jazzy / Humble*
