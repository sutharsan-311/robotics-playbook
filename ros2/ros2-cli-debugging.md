# ROS2 Debugging with ros2 CLI Tools

The `ros2` command-line interface is your first line of defence when a system misbehaves — before you reach for a debugger or rewrite anything. Knowing which subcommand reveals what can halve your time-to-diagnosis.

## What it is

The `ros2` CLI ships with every desktop install and exposes subcommands that map directly onto the ROS2 communication graph: `node`, `topic`, `service`, `action`, `param`, `bag`, and `doctor`. Together they let you inspect live systems without modifying source code — subscribing to topics, querying node metadata, probing services, reading/writing parameters, and validating environment health, all from the shell.

## How it works

The workflow follows a consistent funnel: list → info → echo/call.

### 1. Find what's running

```bash
ros2 node list          # all active nodes
ros2 topic list -t      # all topics with their type
ros2 service list -t    # all services with their type
ros2 action list -t     # all action servers with their type
```

### 2. Inspect a specific entity

```bash
# Full node graph: publishers, subscribers, services, actions
ros2 node info /turtlesim

# Topic subscriber/publisher count + QoS profiles of every endpoint
ros2 topic info -v /turtle1/cmd_vel

# List all parameters on a node; get or set one
ros2 param list /turtlesim
ros2 param get /turtlesim background_r
ros2 param set /turtlesim background_r 150
```

### 3. Observe live data

```bash
ros2 topic echo /turtle1/cmd_vel          # stream all messages
ros2 topic echo --once /turtle1/pose      # grab a single message
ros2 topic hz /scan                       # measured publish rate
ros2 topic bw /camera/image_raw           # bandwidth in bytes/s
ros2 topic delay /odom                    # header.stamp vs wall-clock lag
```

### 4. Inject test messages

```bash
# Publish a one-shot message (--once); use --rate N for continuous
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

### 5. Remap at launch time

When a node publishes on the wrong namespace, fix it without editing source:

```bash
ros2 run turtlesim turtlesim_node --ros-args -r /turtle1/cmd_vel:=/robot/cmd_vel
```

(Source: [docs.ros.org — Passing ROS arguments to nodes via the command-line](https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html))

### 6. Validate the whole setup

```bash
ros2 doctor           # summary: warnings and errors
ros2 doctor --report  # full detail: network config, DDS middleware, env vars
```

`ros2 doctor` checks platform, Python/RMW versions, multicast availability, and environment variables such as `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`. The `--report` flag emits five sections — NETWORK CONFIGURATION, PLATFORM INFORMATION, RMW MIDDLEWARE, ROS 2 INFORMATION, TOPIC LIST — which is the fastest way to rule out environment drift between machines.

## Common pitfalls

**1. Silent QoS mismatches.**
`ros2 topic list` shows both a publisher and subscriber, but no messages arrive. The culprit is almost always a QoS incompatibility — a `BEST_EFFORT` publisher paired with a `RELIABLE` subscriber (or a `VOLATILE` publisher with a `TRANSIENT_LOCAL` subscriber). Run `ros2 topic info -v /your_topic` and compare the QoS profiles of every endpoint. Mismatched durability is particularly sneaky because the connection is listed as established.

**2. Wrong `ROS_DOMAIN_ID`.**
Nodes on different domain IDs are completely invisible to each other. If `ros2 node list` returns nothing but you know a node is running, check that every terminal and launch file exports the same `ROS_DOMAIN_ID` (default is `0`). `ros2 doctor` will flag this.

**3. `ros2 topic hz` lying about rate.**
`ros2 topic hz` reports the *receiving* rate at your machine's subscriber — not the publisher's actual rate. Network congestion, large message types, or a mis-set `--qos-reliability reliable` flag on the measurement subscription can all cause it to under-report. Cross-check with `ros2 topic bw` and inspect `header.stamp` skew with `ros2 topic delay` to distinguish a slow publisher from a lossy transport.

## Further reading

- [Introspection with command line tools — ROS 2 Humble docs](https://docs.ros.org/en/humble/Concepts/Basic/About-Command-Line-Tools.html)
- [Using ros2doctor to identify issues — ROS 2 Humble docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html)
- [Passing ROS arguments to nodes via the command-line — ROS 2 Rolling docs](https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html)

---
*2026-10-17 | ROS2 version: Jazzy / Humble*
