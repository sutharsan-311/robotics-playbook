# ROS2 Services — Request and Response

Services are ROS2's synchronous communication primitive: one node calls, another answers. Use them for operations that require a result before proceeding — think parameter queries, mode switches, or on-demand computations.

## What it is

A ROS2 service is a **request/response** channel between a client node and a server node. Unlike topics (fire-and-forget publish/subscribe), a service call blocks until the server replies or times out. Each service has a **type** defined in a `.srv` file with two sections separated by `---`: the request fields above, the response fields below.

The canonical example from `example_interfaces`:

```
int64 a
int64 b
---
int64 sum
```

Service types are discovered at runtime via DDS; clients and servers match by name (e.g. `add_two_ints`) and type. One server, many clients — but only one response per call.

## How it works

**Server side** — register a callback via `create_service()`. The callback receives a populated request object and must return a populated response object.

```python
# Source: github.com/ros2/examples — rclpy/services/minimal_service/
#         examples_rclpy_minimal_service/service_member_function.py

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_service = MinimalService()
            rclpy.spin(minimal_service)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
```

**Client side** — call `call_async()` to get a `Future`, then drive the executor with `spin_until_future_complete()` to let the response arrive.

```python
# Source: github.com/ros2/examples — rclpy/services/minimal_client/
#         examples_rclpy_minimal_client/client_async_member_function.py

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        return self.cli.call_async(self.req)


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_client = MinimalClientAsync()
            future = minimal_client.send_request()
            rclpy.spin_until_future_complete(minimal_client, future)
            response = future.result()
            minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (minimal_client.req.a, minimal_client.req.b, response.sum))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
```

**Inspecting services from the CLI:**

```bash
ros2 service list                          # all active services
ros2 service type /add_two_ints            # prints example_interfaces/srv/AddTwoInts
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

## Common pitfalls

**1. Deadlock when calling a service from inside a callback**
`spin_until_future_complete()` inside a subscription, timer, or service callback will deadlock the single-threaded executor — it is busy waiting for a response it can never dispatch. The official ROS2 docs are explicit: never call `spin_until_future_complete()` from within a callback in a single-threaded executor. Use a `MultiThreadedExecutor` with a `ReentrantCallbackGroup`, or restructure the logic to chain futures instead of blocking.

**2. Skipping `wait_for_service()`**
Calling `call_async()` before the server is up returns a future that never resolves. Always poll with `wait_for_service(timeout_sec=...)` before sending the first request, especially during node startup races.

**3. Using services for high-frequency data**
Services carry meaningful per-call overhead (DDS discovery, connection setup on first call, RMW round-trip). Streaming sensor data or control loops at >10 Hz via services will saturate the executor and inflate latency. Use a topic with a subscriber instead; reserve services for discrete, low-frequency operations.

## Further reading

- [Writing a simple service and client (Python) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Synchronous vs. asynchronous service clients — ROS 2 docs](https://docs.ros.org/en/jazzy/How-To-Guides/Sync-Vs-Async.html)
- [ros2/examples — minimal_service and minimal_client (GitHub)](https://github.com/ros2/examples/tree/master/rclpy/services)

---
*2026-05-27 | ROS2 version: Jazzy / Humble*
