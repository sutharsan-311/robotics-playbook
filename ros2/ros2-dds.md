# ROS2 DDS — The Communication Layer

DDS (Data Distribution Service) is the publish-subscribe middleware that replaced ROS1's custom TCPROS transport. Understanding it is essential for debugging discovery failures, tuning multi-robot networks, and choosing the right middleware vendor for your deployment.

## What it is

ROS2 delegates all inter-process communication to a DDS implementation. Rather than locking you into one vendor, ROS2 adds a thin abstraction called the **ROS Middleware Interface (RMW)** — a C API that every DDS vendor implements as a plugin. The ROS Client Library (RCL) talks only to RMW; the concrete DDS library lives below it.

The current default vendor per distro:

| Distro | Default RMW |
|---|---|
| Humble | `rmw_fastrtps_cpp` (Fast DDS) |
| Jazzy | `rmw_fastrtps_cpp` (Fast DDS) |
| Rolling | `rmw_fastrtps_cpp` (Fast DDS) |

> **Note:** Galactic briefly defaulted to CycloneDDS but later distros reverted to Fast DDS. Check your distro's release notes if you're unsure.

Other production-ready options: **Eclipse Cyclone DDS** (`rmw_cyclonedds_cpp`) and **Zenoh** (`rmw_zenoh_cpp`, experimental in Jazzy).

### Entity mapping

Every ROS communication primitive maps one-to-one to a DDS entity inside the RMW layer:

| ROS2 | DDS |
|---|---|
| Publisher | DataWriter |
| Subscription | DataReader |
| Topic | Topic + ContentFilteredTopic |
| Node namespace | DDS Participant |

There is no ROS master. Discovery is peer-to-peer: participants broadcast their endpoints over UDP multicast on startup and respond to unicast probes. This is why `ros2 node list` works without `roscore` — the RMW layer does the bookkeeping.

## How it works

### Switching the RMW implementation

Set `RMW_IMPLEMENTATION` before launching any node. All nodes that need to talk to each other **must use the same RMW**:

```bash
# Use CycloneDDS for this session
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launch with an inline override
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch my_pkg bringup.launch.py
```

Install the alternative before switching:
```bash
# Humble example
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### QoS at the DDS layer

DDS exposes a rich Quality of Service API; ROS2 surfaces the most useful subset via `QoSProfile`. The following pattern — taken from the [`ros2/demos` topic_monitor package](https://github.com/ros2/demos/blob/master/topic_monitor/topic_monitor/scripts/data_publisher.py) — shows runtime QoS selection in Python:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

qos_profile = QoSProfile(depth=10)
# Switch to best-effort for sensor streams where dropping a frame is acceptable
qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
```

For a "latched" topic (subscriber receives the last message even if it connects after the publisher):

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

latched_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```
*(Pattern documented in [ROS 2 QoS design article](https://design.ros2.org/articles/qos.html) and used across the ROS2 ecosystem.)*

For a deep-dive into every QoS axis (deadline, liveliness, lifespan), see the companion article `ros2-qos.md`.

### Domain isolation

Every DDS deployment belongs to a numeric **domain ID** (default: `0`). Nodes on the same network with different domain IDs are completely invisible to each other. Use this to isolate robot fleets:

```bash
export ROS_DOMAIN_ID=42   # values 0–101 and 215–232 are safe
```

Restrict discovery to localhost during development to avoid collisions with teammates:
```bash
export ROS_LOCALHOST_ONLY=1   # available since Eloquent
```

## Common pitfalls

**1. Mixed RMW implementations silently break discovery.**
If one node uses Fast DDS and another uses CycloneDDS, they cannot see each other — `ros2 topic list` will show each node's own topics but not the other's. There is no warning. Always verify with `ros2 doctor` or `ros2 wtf` that all nodes agree on the RMW.

**2. CycloneDDS participant index exhaustion on Jazzy.**
The default `rmw_cyclonedds_cpp` on Jazzy caps the participant index at ~32. Systems running many nodes (composite node launchers, lifecycle managers, Nav2 stacks) hit "Failed to find a free participant index" and nodes refuse to start. Fix via a CycloneDDS XML profile:

```xml
<!-- cyclone_config.xml -->
<CycloneDDS>
  <Domain>
    <Internal>
      <ParticipantIndex>none</ParticipantIndex>
    </Internal>
  </Domain>
</CycloneDDS>
```
```bash
export CYCLONEDDS_URI=file:///path/to/cyclone_config.xml
```

**3. Multi-interface machines break CycloneDDS discovery.**
CycloneDDS uses a single network interface for DDS traffic. On machines with multiple active interfaces (e.g., a robot with both WiFi and an internal Ethernet ring), discovery traffic may go out the wrong interface and peers never appear. Explicitly pin the interface in your XML config:

```xml
<NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
```

Fast DDS handles multi-interface scenarios more gracefully by default.

## Further reading

- [About Different Middleware Vendors — ROS 2 Docs (Humble)](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Different-Middleware-Vendors.html) — official list of RMW packages, installation commands, and switching instructions
- [ROS 2 Middleware Interface — design.ros2.org](https://design.ros2.org/articles/ros_middleware_interface.html) — architectural rationale for why ROS2 adopted DDS and how the RMW API is structured
- [ros2/rmw_cyclonedds — GitHub](https://github.com/ros2/rmw_cyclonedds) — source for the CycloneDDS RMW layer; the README contains tuning guidance and known limitations

---

*2026-09-01 | ROS2 versions: Humble / Jazzy*
