# ROS2 Lifecycle Nodes

Lifecycle nodes (managed nodes) give you deterministic control over when a ROS2 node initializes hardware, starts publishing, and shuts down — making them the right choice for anything that touches real hardware or requires coordinated startup sequencing.

## What it is

A `LifecycleNode` wraps a standard ROS2 node in a finite state machine defined in [REP-2000 / the managed nodes design](https://design.ros2.org/articles/node_lifecycle.html). Instead of a node doing everything in its constructor and running forever, it exposes explicit lifecycle transitions that an orchestrator (launch file, supervisor node, or CLI) can trigger and monitor.

**Primary states** (steady, where the node spends most of its time):

| State | Meaning |
|---|---|
| `unconfigured` | Freshly created; no resources allocated |
| `inactive` | Configured; resources ready but not publishing/acting |
| `active` | Fully operational |
| `finalized` | Shutdown complete; node will be destroyed |

**Transition states** (transient, one per callback):

`configuring` → `activating` → `deactivating` → `cleaningup` → `shuttingdown`

Each transition state runs a callback. Your return value determines the outcome:
- `SUCCESS` → advance to the next primary state
- `FAILURE` → roll back to the previous primary state
- `ERROR` (or uncaught exception) → enter `errorprocessing`

## How it works

Your node inherits from `rclcpp_lifecycle::LifecycleNode` instead of `rclcpp::Node` and overrides the transition callbacks. The following is taken directly from the official [`ros2/demos` lifecycle_talker](https://github.com/ros2/demos/blob/rolling/lifecycle/src/lifecycle_talker.cpp):

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {}

  // Called during `configuring`: allocate resources but don't start yet.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    pub_ = this->create_publisher<example_interfaces::msg::String>("lifecycle_chatter", 10);
    timer_ = this->create_wall_timer(1s, [this]() { return this->publish(); });
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return CallbackReturn::SUCCESS;
  }

  // Called during `activating`: enable managed entities to start publishing.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_activate(state);   // activates LifecyclePublisher automatically
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    return CallbackReturn::SUCCESS;
  }

  // Called during `deactivating`: stop publishing without releasing resources.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
    return CallbackReturn::SUCCESS;
  }

  // Called during `cleaningup`: release all resources, return to unconfigured.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

Note that publishers must be `rclcpp_lifecycle::LifecyclePublisher` — a drop-in replacement that silently drops messages when the node is not `active`. You can call `pub_->publish()` unconditionally; the publisher gates transmission based on its activation state.

**Controlling state from the CLI:**

```bash
# Inspect available transitions
ros2 lifecycle get /lifecycle_talker
ros2 lifecycle list /lifecycle_talker

# Drive the state machine
ros2 lifecycle set /lifecycle_talker configure
ros2 lifecycle set /lifecycle_talker activate
ros2 lifecycle set /lifecycle_talker deactivate
ros2 lifecycle set /lifecycle_talker cleanup
```

Every lifecycle node also auto-exposes five services: `__get_state`, `__change_state`, `__get_available_states`, `__get_available_transitions`, and a topic `__transition_event` for monitoring.

## Common pitfalls

**1. Blocking work inside transition callbacks.** Transition callbacks run on the node's executor thread. Heavy initialization (device driver boot, calibration routines) blocks the entire executor, preventing other callbacks from firing. Offload long work to a thread and return `SUCCESS` only after joining, or use a dedicated executor for the lifecycle node.

**2. Forgetting to call `LifecycleNode::on_activate(state)` in `on_activate`.** The base class method iterates over all managed entities (publishers, etc.) and activates them. If you override `on_activate` without calling the parent, your `LifecyclePublisher` stays inactive and silently drops every message — no error, no warning.

**3. Using a regular `rclcpp::Publisher` instead of `rclcpp_lifecycle::LifecyclePublisher`.** A plain publisher ignores lifecycle state entirely and publishes unconditionally. This defeats the purpose of managed nodes and is a common porting mistake when converting existing nodes to lifecycle nodes.

## Further reading

- [Managing node lifecycles — ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/Tutorials/Demos/Managed-Nodes.html) — official tutorial with the full talker/listener/service-client demo
- [Managed nodes design article](https://design.ros2.org/articles/node_lifecycle.html) — the canonical state machine specification with full transition table
- [`ros2/demos` lifecycle source](https://github.com/ros2/demos/tree/rolling/lifecycle) — complete runnable example: talker, listener, and service client in C++

---
*2026-07-08 | ROS2 versions: Jazzy / Humble*
