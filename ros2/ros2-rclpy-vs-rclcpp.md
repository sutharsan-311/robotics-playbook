# ROS2 rclpy vs rclcpp — Python vs C++

Both are official ROS 2 client libraries with a deliberately symmetric API — but they differ at the runtime level in ways that matter for high-bandwidth pipelines, real-time control, and production deployments. Knowing *where* the asymmetry lives prevents silent performance cliffs and architecture mistakes.

---

## Architecture — where the two stacks diverge

Both `rclcpp` and `rclpy` sit on top of `rcl`, a common C library that implements core ROS 2 concepts (nodes, topics, services, parameters, timers, actions). The language-specific layers add idiomatic wrappers on top of `rcl` and the `rosidl` generated message API.

```
rclpy (Python)     rclcpp (C++)
     │                  │
     ▼                  ▼
 Python↔C           C++ structs
  convert            (no copy)
     │                  │
     └────────┬─────────┘
              ▼
             rcl  (C)
              │
             rmw  (C)
              │
             DDS
```

**rclcpp** binds directly to `rcl` using C++ types. `rosidl` generates a native C++ struct for every message type; no conversion occurs between the application layer and the middleware.

**rclpy** adds a Python layer. Every message starts as a Python object. On `publish()`, the Python object is **serialised to its C counterpart** via a pybind11 binding before being handed to `rcl`. The inverse conversion happens on every subscription callback. This round-trip is the primary source of rclpy's overhead on high-throughput topics — it happens regardless of message size.

*(Source: [rclpy package — rclpy 10.0.4 documentation, Rolling](https://docs.ros.org/en/ros2_packages/rolling/api/rclpy/rclpy.html))*

---

## Side-by-side: minimal publisher

**rclpy** (from [`ros2/examples` — humble branch](https://github.com/ros2/examples/blob/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py)):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**rclcpp** (from [`ros2/examples` — humble branch](https://github.com/ros2/examples/blob/humble/rclcpp/topics/minimal_publisher/lambda.cpp)):

```cpp
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(this->count_++);
      this->publisher_->publish(message);
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
}
```

The API is intentionally symmetric. The real divergence is under the hood.

---

## Performance

### Publish overhead

Publishing a 10 MB `PointCloud2` message takes approximately **2.8 ms in rclcpp and 92 ms in rclpy** — a ~33× difference — because the Python→C conversion scales with message size ([ros2/rclpy#763](https://github.com/ros2/rclpy/issues/763)). A related tracking issue ([ros2/ros2#1499](https://github.com/ros2/ros2/issues/1499)) confirms that even in Iron, rclpy pub/sub for large messages remains orders of magnitude slower.

At 30 Hz lidar rates on a typical 360° scan (~5–10 MB), rclpy's publish path alone would saturate available CPU time long before the rest of the pipeline.

For small `std_msgs` (string, int, bool), the overhead is dominated by call overhead rather than copy size, and both libraries perform similarly in the microsecond range.

### Executor threading and the GIL

Both libraries expose `SingleThreadedExecutor` and `MultiThreadedExecutor`. rclcpp's `MultiThreadedExecutor` creates real OS threads; callbacks in different `ReentrantCallbackGroup`s run in true parallel.

rclpy's `MultiThreadedExecutor` also creates real OS threads but is constrained by the **CPython GIL** — at most one thread executes Python bytecode at any instant. CPU-bound Python callbacks (numpy, PIL, custom math) receive **zero parallelism** even with 8 threads configured. The GIL is released during:
- I/O-bound blocking (socket read, subprocess wait)
- `async_send_request` future waits
- C extensions that explicitly drop the GIL (numpy array ops, OpenCV, `tf2_ros.Buffer` lookups)

For CPU-bound workloads in Python, use `concurrent.futures.ProcessPoolExecutor` or restructure the heavy path as an rclcpp component.

---

## Intra-process communication (rclcpp only)

When two rclcpp nodes run in the **same process**, ROS 2 can bypass DDS entirely and pass messages via shared ring buffers. Publishing with `std::unique_ptr<T>` enables zero-copy: ownership of the message is moved into the subscription's queue with no data copy.

```cpp
// Enable IPC at node construction
rclcpp::NodeOptions options;
options.use_intra_process_comms(true);

auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
  "/cloud", rclcpp::QoS(10));

// Zero-copy publish: ownership transferred, no copy
auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
// ... fill msg ...
pub->publish(std::move(msg));
```

When the first subscriber receives the message, the `unique_ptr` is promoted to `shared_ptr`; all subsequent subscribers in the same process share ownership of the same allocation.

*(Source: [Setting up efficient intra-process communication — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Demos/Intra-Process-Communication.html))*

> **rclpy limitation:** Intra-process communication is not available in rclpy. rclpy nodes always go through DDS serialisation even in a single process. This is the second major performance gap after the Python→C conversion overhead ([ros2/design#251](https://github.com/ros2/design/issues/251)).

---

## Composable nodes (rclcpp only)

A composable node is an rclcpp node compiled as a shared library plugin rather than a standalone executable. Multiple composable nodes can be loaded into a single **component container** process at launch time, sharing DDS participant overhead and enabling intra-process communication between them.

```cpp
// my_component.cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class MyComponent : public rclcpp::Node {
public:
  explicit MyComponent(const rclcpp::NodeOptions & options)
  : Node("my_component", options) {}
};

// Registration macro — must appear once per component per library
RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

```python
# launch file — load two components into one container
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='my_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(package='my_pkg', plugin='MyComponent', name='comp_a'),
        ComposableNode(package='my_pkg', plugin='OtherComponent', name='comp_b'),
    ],
)
```

*(Source: [Composing multiple nodes in a single process — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html))*

**rclpy nodes cannot be compiled as components.** rclpy nodes always run as standalone processes or within a `MultiThreadedExecutor` in a single Python process. For pipelines where IPC throughput is critical (sensor driver → filter → estimator), all nodes in the hot path should be rclcpp components in a shared container.

---

## Zero-copy loaned messages (rclcpp only)

Beyond intra-process IPC, rclcpp supports **loaned messages** — the middleware allocates the message buffer in shared memory and loans a pointer to the publisher. The application fills the buffer in-place; no copy happens at the DDS layer either.

```cpp
// Only works if the RMW supports it (Fast DDS with SHM, iceoryx)
if (publisher->can_loan_messages()) {
  auto loaned = publisher->borrow_loaned_message();
  loaned.get().data = 42.0f;
  publisher->publish(std::move(loaned));
}
```

Supported by `rmw_fastrtps_cpp` with the SHM transport profile and `rmw_iceoryx` (Eclipse iceoryx). Not supported by CycloneDDS or Zenoh RMW.

*(Source: [Zero Copy via Loaned Messages — design.ros2.org](https://design.ros2.org/articles/zero_copy.html); [Configure Zero Copy Loaned Messages — Vulcanexus Jazzy docs](https://docs.vulcanexus.org/en/jazzy/ros2_documentation/source/How-To-Guides/Configure-ZeroCopy-loaned-messages.html))*

---

## Lifecycle nodes

Both libraries implement managed (lifecycle) nodes, but the APIs differ subtly.

**rclcpp** uses `rclcpp_lifecycle::LifecycleNode` — a separate class that inherits node interfaces and adds the state machine. Lifecycle publishers (`LifecyclePublisher`) automatically activate/deactivate with the node; `on_activate()` and `on_deactivate()` transition publishers without manual calls.

**rclpy** uses `rclpy.lifecycle.LifecycleNode` with a `LifecycleNodeMixin`. The Python API does not exactly mirror rclcpp ([ros2/rclcpp#898](https://github.com/ros2/rclcpp/issues/898) tracks the gap); in particular, lifecycle-managed timers work differently, and some transition callback signatures differ between versions.

```python
# rclpy lifecycle node skeleton
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

class MyLifecycleNode(LifecycleNode):
    def on_configure(self, state):
        self.get_logger().info('Configuring')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating')
        return TransitionCallbackReturn.SUCCESS
```

*(Source: [rclpy.lifecycle package — Rolling API](https://docs.ros.org/en/rolling/p/rclpy/rclpy.lifecycle.html))*

---

## Actions

Both libraries support actions with similar APIs. The conceptual model is identical: goal, feedback stream, result. The implementation uses asynchronous services underneath.

**rclpy action server** (pattern from [ROS 2 Jazzy action tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)):

```python
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci', self.execute_callback)

    async def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        # ... compute, publish feedback ...
        goal_handle.succeed()
        return Fibonacci.Result()
```

**rclcpp action server** (pattern from [ROS 2 Jazzy C++ action tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)):

```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/fibonacci.hpp"

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciServer : public rclcpp::Node {
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
public:
  FibonacciServer() : Node("fibonacci_action_server") {
    server_ = rclcpp_action::create_server<Fibonacci>(
      this, "fibonacci",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {
        return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> gh) {
        this->execute(gh); });
  }
  void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> gh) { /* ... */ }
};
```

The rclpy `async def execute_callback` model is cleaner to write; the rclcpp version requires explicit goal/cancel handler lambdas and manual thread management. For action servers in orchestration layers (state machines, planners), rclpy is often the right choice.

---

## Parameters

The parameter APIs are functionally equivalent but differ in type enforcement.

**rclcpp** lets you declare a parameter with an explicit type at declaration time:

```cpp
// Type-enforced at declaration; set attempts with wrong type are rejected
this->declare_parameter<double>("max_speed", 1.5);
double speed = this->get_parameter("max_speed").as_double();

// Validate on change
this->add_on_set_parameters_callback(
  [](const std::vector<rclcpp::Parameter> & params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "max_speed" && p.as_double() > 5.0) {
        result.successful = false;
        result.reason = "max_speed must be <= 5.0";
      }
    }
    return result;
  });
```

**rclpy** uses `ParameterDescriptor` for type constraints:

```python
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

descriptor = ParameterDescriptor(
    type=ParameterType.PARAMETER_DOUBLE,
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0)])
self.declare_parameter('max_speed', 1.5, descriptor)

# Validate on change
self.add_on_set_parameters_callback(self._on_params_changed)

def _on_params_changed(self, params):
    from rcl_interfaces.msg import SetParametersResult
    return SetParametersResult(successful=True)
```

`ParameterEventHandler` — available in rclcpp for monitoring parameter changes on *any* node — was requested but not yet implemented for rclpy as of Jazzy ([ros2/rclpy#1105](https://github.com/ros2/rclpy/issues/1105)). rclpy nodes can only monitor their own parameter changes via `add_on_set_parameters_callback`.

*(Source: [ros2_cookbook — rclcpp parameters, mikeferguson/ros2_cookbook](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/parameters.md))*

---

## Feature comparison matrix

| Feature | rclpy | rclcpp |
|---|---|---|
| Topics, services, actions, parameters | ✅ | ✅ |
| Lifecycle nodes | ✅ (limited API) | ✅ (full API) |
| MultiThreadedExecutor — CPU parallelism | ❌ (GIL) | ✅ |
| EventsExecutor (experimental) | ❌ | ✅ (`rclcpp::experimental`) |
| Intra-process communication (IPC) | ❌ | ✅ |
| Composable nodes / component container | ❌ | ✅ |
| Zero-copy loaned messages | ❌ | ✅ (rmw-dependent) |
| `ParameterEventHandler` (cross-node) | ❌ | ✅ |
| Static type checking (mypy stubs) | ✅ (Jazzy+) | ✅ (native) |
| Real-time / deterministic scheduling | ❌ | ✅ (`SCHED_FIFO` + lock-free) |
| Build system integration | `ament_python` | `ament_cmake` |

---

## When to choose which

| Use case | Choice | Reason |
|---|---|---|
| Sensor driver (LiDAR, camera, IMU) | rclcpp | High-bandwidth; IPC + composability |
| Hardware abstraction layer | rclcpp | Driver latency, loaned messages |
| Control loop ≤1 ms period | rclcpp | Real-time scheduling, no GIL |
| Nav / planning orchestration | rclpy | State machine logic, action APIs |
| ML inference node | rclpy | Python ML ecosystem; use subprocess for heavy GPU work |
| Config management / param server | rclpy | Rapid iteration; scripting-friendly |
| Diagnostic / monitoring node | rclpy | Low-frequency; minimal overhead |
| Cross-team rapid prototyping | rclpy | Faster iteration cycle |

**Common production pattern:** sensor drivers, estimators, and controllers in rclcpp components inside a shared container; orchestration, planning, and ML nodes in rclpy processes communicating over DDS.

---

## Common pitfalls

**1. `MultiThreadedExecutor` in rclpy that still runs single-threaded.**
The GIL serialises all CPU-bound Python callbacks regardless of thread count. Verify actual CPU utilisation with `htop -H`. For true Python parallelism across callbacks, use separate processes with `multiprocessing` or `ProcessPoolExecutor` and bridge them over ROS topics.

**2. Default `MutuallyExclusiveCallbackGroup` negates multithreading.**
In both libraries, every entity is assigned to the node's default `MutuallyExclusiveCallbackGroup`. A `MultiThreadedExecutor` with this configuration is functionally identical to `SingleThreadedExecutor`. Explicitly assign `ReentrantCallbackGroup` or separate `MutuallyExclusive` groups to entities that should run concurrently.

**3. Using rclpy for high-bandwidth topics.**
The Python↔C serialisation on every `publish()` and subscription callback scales linearly with message size. For `sensor_msgs/PointCloud2`, `sensor_msgs/Image`, or any large array type at >5 Hz, rclcpp (with optional IPC) is the only viable choice. Confirm expected throughput with a benchmark before committing to rclpy for sensor-adjacent nodes.

**4. Assuming composable node benefits apply to rclpy.**
rclpy nodes cannot participate in a component container and do not benefit from intra-process communication. A system architecture that puts an rclpy node between two rclcpp components that otherwise share a container pays two DDS serialisation round-trips (rclcpp → DDS → rclpy → DDS → rclcpp).

**5. rclpy lifecycle API not matching rclcpp.**
Timer lifecycle management, some transition callback signatures, and `LifecyclePublisher` activation behaviour differ between rclcpp and rclpy. Always test lifecycle state transitions explicitly when porting lifecycle logic from C++ to Python or vice versa. ([ros2/rclcpp#898](https://github.com/ros2/rclcpp/issues/898))

**6. `ParameterEventHandler` unavailable in rclpy.**
rclcpp nodes can subscribe to parameter change events on *other* nodes using `ParameterEventHandler`. rclpy cannot do this — it can only monitor its own parameters. Nodes that need to react to another node's parameter changes from Python must subscribe to `/parameter_events` manually and parse the `rcl_interfaces/msg/ParameterEvent` message.

**7. Mixing rclpy nodes in the same process without a shared executor.**
Each `rclpy.spin()` call blocks the calling thread. Running two rclpy nodes concurrently in one process requires `MultiThreadedExecutor.add_node(node1)` + `executor.add_node(node2)` and a single `executor.spin()`. Spawning a `threading.Thread(target=rclpy.spin, args=(node,))` per node serialises at the GIL and has a known bug where `rclpy.spin_once(node)` detaches the node from its executor. ([rclpy#1445](https://github.com/ros2/rclpy/issues/1445))

---

## Further reading

- [Writing a simple publisher and subscriber (Python) — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) — canonical rclpy publisher/subscriber walkthrough for Jazzy
- [Writing a simple publisher and subscriber (C++) — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) — canonical rclcpp equivalent
- [Setting up efficient intra-process communication — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Demos/Intra-Process-Communication.html) — full demo: `unique_ptr` zero-copy, multi-subscription promotion to `shared_ptr`
- [Composing multiple nodes in a single process — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Composition.html) — component container patterns, launch integration, dynamic vs static composition
- [Zero Copy via Loaned Messages — design.ros2.org](https://design.ros2.org/articles/zero_copy.html) — design rationale for `LoanedMessage` API, iceoryx integration, RMW requirements
- [rclpy.lifecycle package — Rolling API docs](https://docs.ros.org/en/rolling/p/rclpy/rclpy.lifecycle.html) — `LifecycleNode`, `LifecycleNodeMixin`, `TransitionCallbackReturn` API reference
- [rclpy issue #763 — Publishing large data is 30x–100x slower than rclcpp](https://github.com/ros2/rclpy/issues/763) — root cause analysis of the Python↔C serialisation overhead and current status
- [Intra-Process Communications for all language clients — ros2/design#251](https://github.com/ros2/design/issues/251) — open design discussion on bringing IPC to rclpy
- [rclpy issue #1105 — ParameterEventHandler for rclpy](https://github.com/ros2/rclpy/issues/1105) — tracked request for cross-node parameter monitoring in Python
- [ros2_cookbook — rclcpp parameters (mikeferguson)](https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/parameters.md) — practical rclcpp parameter patterns: declare, get, validate, ParameterEventHandler

---

*2026-10-03 | ROS 2: Jazzy / Humble*
