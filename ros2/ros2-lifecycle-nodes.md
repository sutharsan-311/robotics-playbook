# ROS2 Lifecycle Nodes — Managed Nodes and State-Machine Control

Lifecycle nodes (managed nodes) give you deterministic control over when a ROS 2 node initialises hardware, starts publishing, and shuts down — and let an external orchestrator coordinate multi-node startup sequences safely. They are the correct design for anything that touches real hardware, opens device drivers, or must be hot-restartable without killing the entire process.

---

## What it is

A `LifecycleNode` wraps a standard ROS 2 node in a finite state machine defined in [REP-2000 / the managed nodes design](https://design.ros2.org/articles/node_lifecycle.html). Instead of a node doing everything in its constructor and running forever, it exposes explicit lifecycle transitions that a launch file, a supervisor node, or the CLI can trigger and monitor.

**Primary states** (steady, where the node spends most of its time):

| State | What the node does here |
|---|---|
| `unconfigured` | Freshly created; no resources allocated |
| `inactive` | Configured; resources ready but not publishing or acting |
| `active` | Fully operational — publishers emit, timers fire |
| `finalized` | Shutdown complete; node will be destroyed |

**Transition states** (transient, one per callback):

`configuring` → `activating` → `deactivating` → `cleaningup` → `shuttingdown`

Each transition state runs exactly one callback. Your return value drives the outcome:

| Return value | Effect |
|---|---|
| `SUCCESS` | Advance to the next primary state |
| `FAILURE` | Roll back to the previous primary state |
| `ERROR` (or uncaught exception) | Enter `errorprocessing` |

---

## Automatic communication interface

Every lifecycle node auto-exposes these on the ROS graph (where `<node_name>` is the FQN):

| Interface | Type | Purpose |
|---|---|---|
| `<node_name>/get_state` | Service (`GetState`) | Query current primary state |
| `<node_name>/change_state` | Service (`ChangeState`) | Trigger a transition by ID |
| `<node_name>/get_available_states` | Service | List all reachable states |
| `<node_name>/get_available_transitions` | Service | List valid transitions from current state |
| `<node_name>/transition_event` | Topic (`TransitionEvent`) | Broadcast every state change |

The `transition_event` topic is what the `lifecycle_listener` and Nav2's lifecycle manager subscribe to when monitoring a fleet of managed nodes.

---

## C++ implementation

Inherit from `rclcpp_lifecycle::LifecycleNode` and override the six transition callbacks. The following is taken directly from the [`ros2/demos` lifecycle_talker](https://github.com/ros2/demos/blob/jazzy/lifecycle/src/lifecycle_talker.cpp) (Apache 2.0):

```cpp
// Source: github.com/ros2/demos/blob/jazzy/lifecycle/src/lifecycle_talker.cpp
// (Apache 2.0 — ros2/demos)
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {}

  // on_configure: allocate resources (create publisher, timer) but don't start yet.
  // SUCCESS → "inactive"   FAILURE → "unconfigured"   ERROR → "errorprocessing"
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
    timer_ = this->create_wall_timer(1s, [this]() { return this->publish(); });
    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return CallbackReturn::SUCCESS;
  }

  // on_activate: enable managed entities to begin publishing.
  // The base class method activates the LifecyclePublisher automatically.
  // SUCCESS → "active"   FAILURE → "inactive"
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_activate(state);   // activates pub_ — must call this
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    return CallbackReturn::SUCCESS;
  }

  // on_deactivate: stop publishing without releasing resources.
  // SUCCESS → "inactive"   FAILURE → "active"
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
  {
    LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
    return CallbackReturn::SUCCESS;
  }

  // on_cleanup: release all resources; return to "unconfigured" so configure can run again.
  // SUCCESS → "unconfigured"   FAILURE → "inactive"
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on cleanup is called.");
    return CallbackReturn::SUCCESS;
  }

  // on_shutdown: called from any primary state (unconfigured, inactive, or active).
  // state.label() tells you which one.
  // SUCCESS → "finalized"
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.", state.label().c_str());
    return CallbackReturn::SUCCESS;
  }

private:
  void publish()
  {
    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle HelloWorld #" + std::to_string(++count);
    // publish() is called unconditionally — LifecyclePublisher silently drops
    // the message when not in active state.
    pub_->publish(std::move(msg));
  }

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;
  auto lc_node = std::make_shared<LifecycleTalker>("lc_talker");

  // Add the node's base interface — not the node directly — to avoid
  // double-registration in some executor implementations.
  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
```

**package.xml / CMakeLists.txt** additions beyond a standard rclcpp node:

```xml
<!-- package.xml -->
<depend>rclcpp_lifecycle</depend>
<depend>lifecycle_msgs</depend>
```

```cmake
# CMakeLists.txt
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(lifecycle_talker src/lifecycle_talker.cpp)
ament_target_dependencies(lifecycle_talker rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)
install(TARGETS lifecycle_talker DESTINATION lib/${PROJECT_NAME})
```

---

## Python implementation

`rclpy.lifecycle.LifecycleNode` mirrors the C++ API. Override the same six callbacks; return `TransitionCallbackReturn.SUCCESS / FAILURE / ERROR`.

```python
# Source: ros2/rclpy — jazzy branch
# (Apache 2.0 — ros2/rclpy)
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String


class LifecycleTalker(LifecycleNode):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._pub: LifecyclePublisher | None = None
        self._timer = None
        self._count = 0

    # ── transition callbacks ──────────────────────────────────────────────────

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Create resources. Called during 'configuring' → leads to 'inactive'."""
        self._pub = self.create_lifecycle_publisher(String, 'lifecycle_chatter', 10)
        self._timer = self.create_timer(1.0, self._publish)
        self.get_logger().info('on_configure() called')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Enable publishing. Base class activates LifecyclePublisher."""
        # super().on_activate() activates all managed entities (e.g. LifecyclePublisher)
        result = super().on_activate(state)
        self.get_logger().info('on_activate() called')
        return result

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Pause publishing without releasing resources."""
        result = super().on_deactivate(state)
        self.get_logger().info('on_deactivate() called')
        return result

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Release all resources; return to unconfigured."""
        if self._timer:
            self._timer.cancel()
            self._timer = None
        self._pub = None
        self.get_logger().info('on_cleanup() called')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Called from any primary state (unconfigured, inactive, active)."""
        if self._timer:
            self._timer.cancel()
        self._pub = None
        self.get_logger().info(f'on_shutdown() called from state: {state.label}')
        return TransitionCallbackReturn.SUCCESS

    # ── main work ─────────────────────────────────────────────────────────────

    def _publish(self) -> None:
        if self._pub is None:
            return
        msg = String()
        self._count += 1
        msg.data = f'Lifecycle HelloWorld #{self._count}'
        # LifecyclePublisher silently drops messages when not active
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = LifecycleTalker('lc_talker')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**`create_lifecycle_publisher()`** returns a `rclpy.lifecycle.LifecyclePublisher` — the Python equivalent of `rclcpp_lifecycle::LifecyclePublisher`. It silently drops messages when the node is not `active`, identical to the C++ version.

---

## CLI — controlling state from the terminal

```bash
# Show current state
ros2 lifecycle get /lc_talker

# List all reachable states
ros2 lifecycle list /lc_talker

# Drive the state machine step by step
ros2 lifecycle set /lc_talker configure
ros2 lifecycle set /lc_talker activate
ros2 lifecycle set /lc_talker deactivate
ros2 lifecycle set /lc_talker cleanup
ros2 lifecycle set /lc_talker shutdown

# Monitor all state transitions in real time
ros2 topic echo /lc_talker/transition_event
```

---

## Monitoring transition events

Any regular node can subscribe to `/<node_name>/transition_event` (type `lifecycle_msgs/msg/TransitionEvent`) to watch state changes. This is how Nav2's lifecycle manager tracks the entire sensor and navigation stack:

```cpp
// Source: github.com/ros2/demos/blob/jazzy/lifecycle/src/lifecycle_listener.cpp (Apache 2.0)
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LifecycleListener : public rclcpp::Node
{
public:
  explicit LifecycleListener(const std::string & node_name)
  : Node(node_name)
  {
    // Subscribe to the managed node's data topic
    sub_data_ = this->create_subscription<std_msgs::msg::String>(
      "lifecycle_chatter", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
      });

    // Subscribe to transition events — fires on every state change
    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "/lc_talker/transition_event", 10,
      [this](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg) {
        RCLCPP_INFO(get_logger(), "Transition from state %s to %s",
          msg->start_state.label.c_str(), msg->goal_state.label.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_data_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr sub_notification_;
};
```

---

## Launch file — `LifecycleNode` action

`launch_ros.actions.LifecycleNode` is a drop-in replacement for the `Node` action. It starts the node in `unconfigured` state; you still need to trigger transitions externally (via the lifecycle service client, Nav2's manager, or a custom script).

The following is the launch file from [`ros2/demos` jazzy](https://github.com/ros2/demos/blob/jazzy/lifecycle/launch/lifecycle_demo_launch.py) (Apache 2.0):

```python
# Source: github.com/ros2/demos/blob/jazzy/lifecycle/launch/lifecycle_demo_launch.py
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Managed node — starts in 'unconfigured'
        LifecycleNode(
            package='lifecycle',
            executable='lifecycle_talker',
            name='lc_talker',
            namespace='',
            output='screen',
        ),
        # Regular subscriber that monitors data and transition events
        Node(
            package='lifecycle',
            executable='lifecycle_listener',
            output='screen',
        ),
        # External controller that drives the lifecycle state machine
        # When the client exits it shuts down the whole launch
        Node(
            package='lifecycle',
            executable='lifecycle_service_client',
            output='screen',
            on_exit=Shutdown(),
        ),
    ])
```

For production bringup, Nav2's `nav2_lifecycle_manager` is the standard orchestrator — it configures and activates a named list of lifecycle nodes in order, monitors their `transition_event` topics, and shuts down the chain on failure.

---

## Programmatic state control — `ChangeState` service

A supervisor node (or the lifecycle service client) drives transitions by calling the `/<node_name>/change_state` service with a `lifecycle_msgs/srv/ChangeState` request. Valid `transition.id` values are defined in `lifecycle_msgs/msg/Transition`:

| Transition ID constant | int | From state → To state |
|---|---|---|
| `TRANSITION_CONFIGURE` | 1 | `unconfigured` → `inactive` |
| `TRANSITION_CLEANUP` | 2 | `inactive` → `unconfigured` |
| `TRANSITION_ACTIVATE` | 3 | `inactive` → `active` |
| `TRANSITION_DEACTIVATE` | 4 | `active` → `inactive` |
| `TRANSITION_ACTIVE_SHUTDOWN` | 7 | `active` → `finalized` |
| `TRANSITION_INACTIVE_SHUTDOWN` | 8 | `inactive` → `finalized` |
| `TRANSITION_UNCONFIGURED_SHUTDOWN` | 9 | `unconfigured` → `finalized` |

```cpp
// Source: github.com/ros2/demos/blob/jazzy/lifecycle/src/lifecycle_service_client.cpp
// Service names follow the convention: <node_name>/get_state and <node_name>/change_state
static constexpr char const * node_get_state_topic    = "lc_talker/get_state";
static constexpr char const * node_change_state_topic = "lc_talker/change_state";

// Clients are plain rclcpp::Client — the lifecycle node services use standard ROS 2 services
client_get_state_    = create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
client_change_state_ = create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
```

The Python equivalent uses `trigger_configure()`, `trigger_activate()`, `trigger_deactivate()`, `trigger_cleanup()`, and `trigger_shutdown()` as convenience wrappers on `LifecycleNodeMixin` — these call `__change_state()` internally and are the idiomatic way to self-transition from within the node.

---

## Common pitfalls

**1. Forgetting `LifecycleNode::on_activate(state)` in `on_activate` (C++).**
The base class method iterates over all managed entities (including `LifecyclePublisher`) and activates them. If you override `on_activate` without calling the parent, your publisher stays inactive and silently drops every message — no error, no warning.

**2. Using `rclcpp::Publisher` instead of `rclcpp_lifecycle::LifecyclePublisher`.**
A plain publisher ignores lifecycle state and publishes unconditionally — including in `inactive` state. This defeats the purpose of managed nodes and is a common porting mistake. Always use `create_publisher<T>` on a `LifecycleNode` (which returns a `LifecyclePublisher`) or explicitly type the member as `rclcpp_lifecycle::LifecyclePublisher<T>::SharedPtr`.

**3. Python: forgetting `super().on_activate(state)` / `super().on_deactivate(state)`.**
In rclpy, the `LifecycleNodeMixin.on_activate()` base implementation iterates over all managed entities registered via `add_managed_entity()` and activates them. If you override without `super()`, any custom managed entities (and the built-in `LifecyclePublisher`) are not activated.

**4. Blocking work inside transition callbacks.**
Transition callbacks run on the node's executor thread. Heavy initialisation (device driver boot, calibration, network connection) blocks the entire executor and prevents other callbacks from firing. Offload the work to a dedicated thread and return `SUCCESS` only after joining — or, for very long setup, return `SUCCESS` immediately and gate the `on_activate` return until setup finishes.

**5. `on_shutdown` is called from any primary state.**
`shutdown` can be triggered from `unconfigured`, `inactive`, or `active`. Your `on_shutdown` implementation must handle resources correctly in all three cases. Inspect `state.label()` (C++) or `state.label` (Python) to decide what to clean up. Calling `timer_.reset()` or `pub_.reset()` on an already-null pointer is safe; calling device teardown code on a device that was never initialised is not.

**6. Not storing `trigger_*` return values (Python self-transitions).**
`trigger_configure()`, `trigger_activate()`, etc. return `bool` — `True` on success. Calling them without checking the return value silently swallows transition failures and leaves the node in an unexpected state.

**7. Expecting `ros2 lifecycle set` to work before the node finishes constructing.**
The lifecycle services are only registered once the node's executor starts spinning. If you race a `ros2 lifecycle set configure` CLI call against node startup you get `Service not available`. In automated launch, use a lifecycle service client node that polls `wait_for_service()` before sending the first request.

---

## Further reading

- [Managing node lifecycles — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Demos/Managed-Nodes.html) — official tutorial with the full talker/listener/service-client demo and expected terminal output
- [Managed nodes design article — design.ros2.org](https://design.ros2.org/articles/node_lifecycle.html) — the canonical state machine specification with the full transition table and rationale
- [`ros2/demos` lifecycle source — jazzy branch (GitHub)](https://github.com/ros2/demos/tree/jazzy/lifecycle) — complete runnable C++ examples: `lifecycle_talker`, `lifecycle_listener`, `lifecycle_service_client`, and the launch file
- [`rclpy.lifecycle` API — Jazzy docs](https://docs.ros.org/en/jazzy/p/rclpy/rclpy.lifecycle.html) — Python `LifecycleNode`, `LifecyclePublisher`, `TransitionCallbackReturn`, and `LifecycleState` reference
- [`ros2/rclpy` lifecycle module source — jazzy (GitHub)](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/lifecycle/node.py) — `LifecycleNodeMixin` implementation: `trigger_*` convenience methods, `add_managed_entity`, and service wiring
- [`nav2_lifecycle_manager` — Nav2 Jazzy docs](https://docs.ros.org/en/jazzy/p/nav2_lifecycle_manager/) — production orchestrator that configures/activates a fleet of lifecycle nodes in dependency order and monitors their health via `transition_event`
- [`lifecycle_msgs` package](https://docs.ros.org/en/jazzy/p/lifecycle_msgs/) — `State`, `Transition`, `TransitionEvent`, `GetState`, `ChangeState` message and service definitions

---

*2026-09-05 | ROS2 version: Jazzy / Humble*
