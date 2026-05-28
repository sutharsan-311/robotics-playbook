# ROS2 Actions — Long Running Tasks

Actions are ROS2's mechanism for long-running, preemptable work with streaming feedback — the right tool when a service's single request/response is too thin and a topic's fire-and-forget is too loose.

## What it is

An **action** is a three-phase interaction between an *action client* and an *action server*:

1. **Goal** — the client sends a goal; the server accepts or rejects it.
2. **Feedback** — while executing, the server streams incremental updates to the client.
3. **Result** — when done (or canceled/aborted), the server sends a final result.

Under the hood, each action is implemented as **three services + one topic**: `goal`, `cancel`, and `result` services, plus a `feedback` topic. The `ros2action` CLI exposes all of this transparently.

The server maintains a per-goal **state machine**:

```
ACCEPTED → EXECUTING → SUCCEEDED
                     ↘ ABORTED
         → CANCELING → CANCELED
```

Rejected goals never enter the state machine at all.

Actions shine for tasks like: navigating to a waypoint, executing a manipulation trajectory, running a multi-step inspection sequence — anything where a caller needs to track progress and retain the ability to cancel.

## How it works

Below is the canonical `execute_callback` from the official [`ros2/demos`](https://github.com/ros2/demos/blob/rolling/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_server.py) repository (Apache 2.0), trimmed to the essential pattern:

```python
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
import rclpy, time
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:      # ← always check this
                goal_handle.canceled()
                return Fibonacci.Result()
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main():
    with rclpy.init():
        server = FibonacciActionServer()
        rclpy.spin(server, executor=MultiThreadedExecutor())  # ← required
```

On the client side, `send_goal_async()` returns a future that resolves to a `ClientGoalHandle`. Call `goal_handle.get_result_async()` from the response callback to chain the result future, and pass a `feedback_callback` to `send_goal_async()` to process streaming updates. The full client is in the same demos repo.

## Common pitfalls

**1. Using `SingleThreadedExecutor` with an action server.**
The `execute_callback` runs in a separate thread but still needs the executor to process incoming cancel requests and feedback publications. With a single-threaded executor the callback blocks all other processing, making cancellation unresponsive. Always spin action servers with `MultiThreadedExecutor` (or assign callbacks to a `ReentrantCallbackGroup`).

**2. Ignoring `is_cancel_requested` inside the execution loop.**
Calling `cancel_callback` with `CancelResponse.ACCEPT` only signals intent. The server is responsible for actually stopping: you must poll `goal_handle.is_cancel_requested` at reasonable intervals inside `execute_callback` and call `goal_handle.canceled()` when true. Skipping this means cancel requests are silently swallowed.

**3. Not checking `goal_handle.accepted` on the client.**
`send_goal_async()` resolves even when a goal is *rejected*. If you call `get_result_async()` on a rejected handle (accepted == False) you'll get a `None` result future and a confusing crash. Always gate on `if not goal_handle.accepted: return` in the response callback.

## Further reading

- [About Actions — ROS 2 Humble docs](https://docs.ros.org/en/humble/Concepts/Basic/About-Actions.html) — architecture, state machine diagram, wire protocol overview.
- [Writing an Action Server and Client (Python) — ROS 2 Humble docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) — step-by-step tutorial building a custom action type from scratch.
- [ros2/demos — action_tutorials_py](https://github.com/ros2/demos/tree/rolling/action_tutorials/action_tutorials_py) — the authoritative reference implementation used in this article; includes server, client, and custom `.action` file.

---
*2026-05-28 | ROS2 version: Jazzy / Humble*
