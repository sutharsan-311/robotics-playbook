# ROS2 Parameters — Configuration at Runtime

Parameters are ROS2's built-in mechanism for exposing node configuration as named, typed values that can be read, written, and monitored at runtime — without recompiling or restarting the node. They replace the ROS1 parameter server with a fully distributed, per-node model enforced by the DDS graph.

## What it is

Every parameter lives *inside* a node (not in a central server). A node declares the parameters it accepts, their types, and default values. External tools — the CLI, a launch file, another node — can then read or override those values. Parameters are strongly typed: `bool`, `int`, `double`, `string`, `byte_array`, and their array variants. Attempting to set a wrong type is rejected by default.

The lifecycle of a parameter is:
1. **Declare** — registers the name, type, default value, and optional metadata at node startup.
2. **Get** — read the current value inside the node's logic.
3. **Set** — change the value at runtime, triggering any registered callbacks.

Parameters are scoped to a node. `ros2 param list` shows parameters per fully-qualified node name, e.g. `/my_robot/camera_node`. There is no global namespace to collide in.

## How it works

### Declaring and reading (Python)

The canonical pattern from the [official Humble tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html):

```python
# Source: docs.ros.org — Using parameters in a class (Python), Humble
import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        my_parameter_descriptor = ParameterDescriptor(
            description='This parameter is mine!'
        )
        self.declare_parameter('my_parameter', 'world', my_parameter_descriptor)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter') \
                       .get_parameter_value().string_value
        self.get_logger().info('Hello %s!' % my_param)
```

### Reacting to changes at runtime

Register an `on_set_parameters_callback` to validate or act on parameter changes without polling. The callback receives a list of `Parameter` objects and must return a `SetParametersResult`:

```python
# Source: docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html
from rcl_interfaces.msg import SetParametersResult

def __init__(self):
    ...
    self.add_on_set_parameters_callback(self.parameters_callback)

def parameters_callback(self, params):
    for param in params:
        if param.name == 'my_parameter' and param.type_ == Parameter.Type.STRING:
            self.get_logger().info('Got new value: %s' % param.value)
    return SetParametersResult(successful=True)
```

This is the ROS2 equivalent of ROS1's `dynamic_reconfigure` — but built directly into the node API with no extra package required.

### Loading parameters from a YAML file

Parameters can be supplied at launch via a YAML file. The structure is mandatory — the key `ros__parameters` (double underscore) is required:

```yaml
# Source: docs.ros.org/en/humble — Understanding parameters tutorial
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 69
    use_sim_time: false
```

Load at node startup:
```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

Or dump and reload a running node's current parameters:
```bash
ros2 param dump /turtlesim > turtlesim.yaml
ros2 param load /turtlesim turtlesim.yaml
```

### Key CLI commands

```bash
ros2 param list /my_node          # list all declared parameters
ros2 param get /my_node speed     # read a value
ros2 param set /my_node speed 1.5 # set at runtime (current session only)
ros2 param describe /my_node speed # show type, description, constraints
```

## Common pitfalls

**1. YAML parameters silently ignored if not declared.**
If a YAML file passes a parameter that the node never calls `declare_parameter()` on, that value is loaded into the node's internal store but won't appear in `ros2 param list` and `get_parameter()` returns false. This is a documented rclcpp behaviour ([issue #2264](https://github.com/ros2/rclcpp/issues/2264)). Always declare every expected parameter, even if just to establish the default.

**2. `ros2 param set` changes are not persistent.**
Setting a value via the CLI only affects the running instance. On restart the node reloads its compiled defaults (or what's in the launch file). For persistence, pass a YAML file at launch or use `ros2 param dump` → edit → `ros2 param load`.

**3. Type mismatches cause hard rejections, not coercions.**
ROS2 won't silently cast `"1.0"` to a `double`. If a YAML value type doesn't match the declared type (e.g. `1` vs `1.0` for a `DOUBLE` parameter), the load will fail or the value will be ignored. Use explicit YAML types: `1.0` not `1` for doubles, `true`/`false` not `1`/`0` for booleans.

## Further reading

- [About Parameters in ROS 2 (Humble)](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html) — official reference covering the parameter API, types, and callbacks
- [Using Parameters in a Class (Python) — ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html) — step-by-step tutorial with runnable code
- [generate_parameter_library (PickNik Robotics)](https://github.com/PickNikRobotics/generate_parameter_library) — declarative parameter generation for C++ nodes: define params in YAML, get type-safe accessors and validators generated automatically

---
*2026-05-29 | ROS2 version: Jazzy / Humble*
