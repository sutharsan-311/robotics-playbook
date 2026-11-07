# ROS2 Debugging with ros2 CLI Tools

The `ros2` CLI is your first line of defence when a system misbehaves — before you reach for a debugger or rewrite anything. It exposes the full communication graph without touching source code: node discovery, topic introspection, service calls, parameter editing, bag recording, log-level tuning, and lifecycle state inspection, all from the shell.

This article covers the full CLI surface area in diagnostic order — from "what is running?" to "why isn't it talking?" to "capture it for offline analysis."

---

## The diagnostic funnel

Every investigation follows the same three-step funnel:

1. **List** — discover what entities exist in the graph
2. **Info** — inspect one entity's connections and QoS
3. **Echo / call** — observe live data or inject test traffic

If you skip to step 3 without steps 1 and 2, you waste time on the wrong topic or the wrong node.

---

## Node introspection

```bash
# All active nodes
ros2 node list

# Full node graph: publishers, subscribers, services, action servers/clients
ros2 node info /turtlesim

# Node info with full QoS details (verbose)
ros2 node info -v /my_robot_driver
```

`ros2 node info` is the fastest way to confirm whether a node has actually advertised the topics you expect. A node that crashes during startup may still appear briefly in `node list` before the daemon clears it — `node info` will return an empty graph in that case.

---

## Topic inspection

### Enumerate and inspect

```bash
ros2 topic list -t            # all active topics with their type
ros2 topic info /scan         # publisher count, subscriber count
ros2 topic info -v /scan      # + full QoS profile of every endpoint
```

`ros2 topic info -v` is the definitive tool for diagnosing silent QoS mismatches (see [Common pitfalls](#common-pitfalls)).

### Stream live data

```bash
ros2 topic echo /turtle1/cmd_vel          # all messages
ros2 topic echo --once /turtle1/pose      # single message then exit

# Print only a specific field (dot-notation path)
ros2 topic echo /odom --field pose.pose.position

# Suppress large arrays to keep output readable (e.g. PointCloud2, Image)
ros2 topic echo /camera/color/image_raw --no-arr
```

`--field` and `--no-arr` are essential when echoing high-bandwidth sensor topics — without them `PointCloud2` or `CompressedImage` data floods the terminal.

### Measure rate and bandwidth

```bash
ros2 topic hz /scan                 # measured publish frequency (Hz)
ros2 topic bw /camera/image_raw     # bandwidth in bytes/s
ros2 topic delay /odom              # header.stamp vs wall-clock lag (needs Header)
```

`ros2 topic hz` reports the *receiving* rate at your machine, not the publisher's true rate. Network congestion or large QoS queues can cause it to under-report. Cross-check with `ros2 topic bw`; use `ros2 topic delay` to quantify latency.

### Inject test traffic

```bash
# One-shot publish
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Continuous at 10 Hz
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

*(Source: [docs.ros.org — Understanding ROS 2 Topics, Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html))*

---

## Service calls

```bash
ros2 service list -t                          # all active services with type

# Call a service
ros2 service call /spawn turtlesim/srv/Spawn \
  '{x: 2.0, y: 2.0, theta: 0.2, name: "turtle2"}'

# Check the interface definition before calling
ros2 interface show turtlesim/srv/Spawn
```

The YAML payload must match the **Request** section of the `.srv` file exactly. `ros2 interface show` prints the separator `---` between request and response so you know which fields you need to fill.

---

## Action debugging

```bash
ros2 action list -t                           # all action servers with type
ros2 action info /navigate_to_pose            # client/server count

# Send a goal and stream feedback
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'

# Send goal and also print feedback while executing
ros2 action send_goal --feedback /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}'
```

---

## Parameter inspection and editing

```bash
ros2 param list /turtlesim                        # all parameters on a node
ros2 param get /turtlesim background_r            # read one value
ros2 param set /turtlesim background_r 150        # write at runtime

# Describe: type, range constraints, description string
ros2 param describe /turtlesim background_r

# Snapshot all parameters to YAML (useful before making changes)
ros2 param dump /turtlesim > turtlesim_params.yaml

# Restore from snapshot at runtime
ros2 param load /turtlesim turtlesim_params.yaml

# Load a parameter file at node startup (affects read-only params too)
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_params.yaml
```

*(Source: [docs.ros.org — Using the ros2 param command-line tool, Jazzy](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros2-param.html))*

`ros2 param dump` before any parameter change gives you a restore point. `ros2 param load` at runtime skips read-only parameters with a warning — load at startup if you need those.

---

## Bag recording for offline analysis

`ros2 bag` captures a live session to disk and replays it deterministically — invaluable for reproducing intermittent bugs or testing algorithms against real sensor data without the hardware.

```bash
# Record all topics
ros2 bag record -a

# Record specific topics only
ros2 bag record -t /scan /odom /cmd_vel

# Record with a named output directory
ros2 bag record -a -o my_robot_session

# Inspect a recording: topic list, message count, duration, file size
ros2 bag info my_robot_session/

# Replay at real time
ros2 bag play my_robot_session/

# Replay at half speed (useful for slow algorithms or debugging timing)
ros2 bag play --rate 0.5 my_robot_session/

# Replay only specific topics from the bag
ros2 bag play my_robot_session/ --topics /scan /odom

# Loop indefinitely (useful for algorithm development)
ros2 bag play --loop my_robot_session/
```

*(Source: [github.com/ros2/rosbag2 — jazzy branch](https://github.com/ros2/rosbag2/tree/jazzy))*

Bags are stored as directories containing an `.mcap` file (default since Humble) plus a `metadata.yaml`. The `.mcap` format supports random-access seeks — `ros2 bag play --start-offset <seconds>` jumps to a time offset without scanning the whole file.

---

## Log level control at runtime

ROS 2 uses five severity levels: `DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL`. The default is `INFO`. You can change the floor level globally or per named logger when launching a node:

```bash
# Set all loggers to DEBUG for this invocation
ros2 run my_pkg my_node --ros-args --log-level debug

# Set a specific named logger to DEBUG, leave others at INFO
ros2 run my_pkg my_node --ros-args --log-level my_node.MotionPlanner:=debug

# Multiple overrides in one command
ros2 run my_pkg my_node --ros-args \
  --log-level debug \
  --log-level my_node.SensorFusion:=info
```

Logger names follow the node name and class hierarchy. The named logger syntax is `logger_name:=level` — note the `:=` separator.

*(Source: [docs.ros.org — Logging and logger configuration, Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html))*

---

## Lifecycle node inspection

Lifecycle nodes go through states (`unconfigured → inactive → active → finalized`) managed by a lifecycle manager. You can inspect and manually drive state transitions from the CLI — useful when a Nav2 server stalls during bring-up.

```bash
# List available transitions from the node's current state
ros2 lifecycle list /my_lifecycle_node

# Query the node's current state
ros2 lifecycle get /my_lifecycle_node

# Trigger a named transition
ros2 lifecycle set /my_lifecycle_node configure
ros2 lifecycle set /my_lifecycle_node activate
ros2 lifecycle set /my_lifecycle_node deactivate
ros2 lifecycle set /my_lifecycle_node cleanup
ros2 lifecycle set /my_lifecycle_node shutdown
```

*(Source: [github.com/ros2/demos — lifecycle/README.rst](https://github.com/ros2/demos/blob/master/lifecycle/README.rst))*

If the lifecycle manager is driving the nodes you want to inspect, call `ros2 lifecycle get` without touching `set` — triggering a transition manually while the manager is active will put the system in an unexpected state.

---

## Environment validation with ros2 doctor

`ros2 doctor` examines your environment and the live graph, then reports warnings and errors. Run it first on a new machine or after any ROS distro change.

```bash
ros2 doctor           # summary: pass / warn / fail count
ros2 doctor --report  # full detail across five sections
```

The `--report` flag emits five sections:

| Section | What it checks |
|---|---|
| `NETWORK CONFIGURATION` | Interface IPs, multicast availability |
| `PLATFORM INFORMATION` | OS, kernel, Python/RMW versions |
| `RMW MIDDLEWARE` | Active RMW implementation, vendor |
| `ROS 2 INFORMATION` | Distribution, package versions |
| `TOPIC LIST` | All active topics and their QoS |

`ros2 wtf` is an alias for `ros2 doctor` — same output, different vibes.

*(Source: [docs.ros.org — ros2doctor package, Jazzy](https://docs.ros.org/en/jazzy/p/ros2doctor/))*

---

## Remapping at launch time

When a node publishes on the wrong namespace, fix it without editing source:

```bash
ros2 run turtlesim turtlesim_node \
  --ros-args -r /turtle1/cmd_vel:=/robot/cmd_vel

# Remap the node name itself
ros2 run turtlesim turtlesim_node \
  --ros-args -r __node:=my_turtlesim
```

*(Source: [docs.ros.org — Passing ROS arguments to nodes via the command-line, Jazzy](https://docs.ros.org/en/jazzy/How-To-Guides/Node-arguments.html))*

---

## Common pitfalls

**1. Silent QoS mismatches.**
`ros2 topic list` shows publisher and subscriber, but no data arrives. The culprit is almost always QoS incompatibility: a `BEST_EFFORT` publisher paired with a `RELIABLE` subscriber, or `VOLATILE` durability vs `TRANSIENT_LOCAL`. Run `ros2 topic info -v /your_topic` and compare profiles. Durability mismatches are particularly insidious because the connection is listed as established, not as an error.

**2. Wrong `ROS_DOMAIN_ID`.**
Nodes on different domain IDs are completely invisible to each other. If `ros2 node list` returns nothing but you know nodes are running, check every terminal, every launch file, and every container for the same `ROS_DOMAIN_ID` (default `0`). `ros2 doctor` flags this explicitly.

**3. `ros2 topic hz` under-reports the actual publish rate.**
`ros2 topic hz` measures the *receiving* rate — network congestion, large message payloads, or a mismatched QoS reliability flag on the measurement subscription all cause it to read low. Cross-check with `ros2 topic bw` and use `ros2 topic delay` to distinguish a slow publisher from a lossy transport path.

**4. `ros2 node list` returns a stale node after crash.**
The ROS 2 daemon caches node discovery. If a node crashes abruptly, it can persist in `node list` until the daemon's lease expires (seconds to tens of seconds). `ros2 node info /crashed_node` will return empty publishers/subscribers, confirming the node is gone despite appearing in the list.

**5. `ros2 param load` silently skips read-only parameters.**
Read-only parameters (`rcl_interfaces/msg/ParameterType` marked `read_only = true`) cannot be changed at runtime. `ros2 param load` prints a warning but continues — the read-only params keep their old values. To apply read-only params, pass the YAML file at node startup with `--ros-args --params-file`.

**6. Calling `ros2 lifecycle set` while the lifecycle manager is active.**
The lifecycle manager sends transitions on a schedule. Manually calling `ros2 lifecycle set` on a managed node at the same time causes a race condition that leaves the node in a transition state the manager did not expect. Use `ros2 lifecycle get` to monitor state; call `set` only when the lifecycle manager is not running.

---

## Further reading

- [Understanding ROS 2 Topics — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) — topic list, echo, hz, bw, delay, pub with worked examples
- [Understanding ROS 2 Parameters — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) — param list, get, set, dump, load
- [Using the ros2 param command-line tool — ROS 2 Jazzy how-to](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros2-param.html) — param describe, dump, load, startup file usage
- [Using ros2doctor to identify issues — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html) — doctor report sections, common failure patterns
- [Logging and logger configuration — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html) — per-logger level overrides, `--log-level` flag, named logger syntax
- [Recording and playing back data — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — `ros2 bag record`, `play`, `info` with examples
- [Lifecycle node demo — ros2/demos (GitHub)](https://github.com/ros2/demos/blob/master/lifecycle/README.rst) — `ros2 lifecycle list/get/set` with the canonical demo node
- [Passing ROS arguments to nodes via the command-line — ROS 2 Jazzy how-to](https://docs.ros.org/en/jazzy/How-To-Guides/Node-arguments.html) — remapping, parameter files, log-level, enclave, all `--ros-args` options
- [ros2cli repository — jazzy branch (GitHub)](https://github.com/ros2/ros2cli/tree/jazzy) — source for all CLI verbs; check here for exact flag names when the docs lag

---

*2026-11-07 | ROS2 version: Jazzy / Humble*
