# ROS2 rclpy vs rclcpp — Python vs C++

Both are official ROS 2 client libraries that expose the same conceptual API — but they differ enough in runtime behaviour that choosing the wrong one for a latency-sensitive or high-bandwidth subsystem will cost you.

---

## What it is

`rclcpp` and `rclpy` are the C++ and Python client libraries for ROS 2. Both sit on top of `rcl`, a common C library that implements core ROS 2 concepts (nodes, topics, services, parameters, timers). The language-specific layers add idiomatic wrappers on top of `rcl` and the `rosidl` message API.

**rclcpp** binds directly to `rcl` using C++ types. Message objects are native C++ structs; no conversion is needed before handing them to the middleware.

**rclpy** adds a Python layer. Every message starts as a Python object and is **converted to its C counterpart** before being passed into `rcl`. That conversion happens on every `publish()` and every subscription callback return — it is the primary source of rclpy's overhead on high-throughput topics.

---

## How it works

### Side-by-side: minimal publisher

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

### Executors and threading

Both libraries expose `SingleThreadedExecutor`, `MultiThreadedExecutor`, and (rclcpp-only) `StaticSingleThreadedExecutor`. The rclcpp `MultiThreadedExecutor` uses real OS threads — callbacks in different `ReentrantCallbackGroup`s run in true parallel. The rclpy `MultiThreadedExecutor` uses Python threads, which are subject to the **GIL** — only one thread executes Python bytecode at a time. I/O-bound callbacks (waiting on a socket, a subprocess) can overlap; CPU-bound callbacks cannot.

### Performance reality check

Publishing a 10 MB `PointCloud2` message takes approximately **2.8 ms in rclcpp and 92 ms in rclpy** — a ~33× difference — due to the Python→C message serialisation cost ([ros2/rclpy#763](https://github.com/ros2/rclpy/issues/763)). At 30 Hz lidar rates, rclpy alone would saturate the publish path.

---

## Common pitfalls

**1. GIL negates MultiThreadedExecutor for CPU-bound work.**  
Switching to `rclpy.executors.MultiThreadedExecutor` with multiple threads gives you parallelism for I/O-bound callbacks only. If your callbacks do numpy math or perception processing, they still serialise at the GIL. Use `concurrent.futures.ProcessPoolExecutor` or move the heavy path to rclcpp.

**2. Default callback group blocks concurrent execution.**  
The default callback group is `MutuallyExclusiveCallbackGroup` in both libraries. If all your subscriptions and timers share it, a `MultiThreadedExecutor` behaves identically to a `SingleThreadedExecutor`. You must explicitly create a `ReentrantCallbackGroup` and assign it when creating subscriptions or timers that should run concurrently.

**3. Mixing rclpy nodes in the same process doesn't share threads.**  
Each `spin()` call blocks. Running two rclpy nodes concurrently requires either `MultiThreadedExecutor.add_node(node1); executor.add_node(node2)` or separate processes. Spawning a `Thread(target=rclpy.spin, args=(node,))` per node is a common mistake that also reintroduces GIL contention.

---

## When to choose which

| Criterion | rclpy | rclcpp |
|---|---|---|
| Rapid prototyping | ✅ | — |
| ML / Python ecosystem integration | ✅ | — |
| High-bandwidth topics (cameras, LiDAR) | — | ✅ |
| Hard real-time / deterministic latency | — | ✅ |
| Hardware drivers | — | ✅ |
| Config/param management nodes | ✅ | — |

A common pattern in production systems: write sensor drivers and control loops in rclcpp, write perception orchestration and high-level planners in rclpy — and let DDS handle the boundary.

---

## Further reading

- [Writing a simple publisher and subscriber (Python) — ROS 2 Humble docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Executors — ROS 2 Humble Concepts](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html)
- [rclpy issue #763 — Publishing large data is 30x-100x slower than rclcpp](https://github.com/ros2/rclpy/issues/763)

---

*2026-09-09 | ROS 2: Jazzy / Humble*
