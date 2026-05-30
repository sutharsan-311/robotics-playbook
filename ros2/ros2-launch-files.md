# ROS2 Launch Files — Orchestrating Systems

A launch file is the standard mechanism for starting a collection of ROS 2 nodes, setting parameters, and wiring up remappings in a single, repeatable command — replacing the brittle shell scripts of ROS 1.

---

## What it is

ROS 2 launch files are declarative system descriptions that tell `ros2 launch` exactly which nodes to start, how to configure them, and how they should communicate. They support three formats — **Python**, XML, and YAML — with Python being the most expressive and widely used.

The entry point for any Python launch file is the `generate_launch_description()` function, which must return a `LaunchDescription` object. The launch system evaluates this at runtime, resolving substitutions, arguments, and conditionals before spinning up processes.

Key primitives:
- **`Node`** — spawns a node process (`launch_ros.actions.Node`)
- **`IncludeLaunchDescription`** — composes other launch files
- **`DeclareLaunchArgument`** / **`LaunchConfiguration`** — parameterise launch from the CLI or parent files
- **`GroupAction`** — scopes remappings and configurations to a subset of nodes

---

## How it works

The canonical example from the official ROS 2 Jazzy docs launches two independent `turtlesim` instances and a mimic node that bridges them via topic remapping:

```python
# turtlesim_mimic_launch.py
# Source: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

What's happening here:
- `namespace` isolates both `turtlesim_node` instances so their topics (`/turtlesim1/turtle1/pose`, `/turtlesim2/turtle1/pose`) don't collide.
- `remappings` rewires the `mimic` node's generic input/output topics to the concrete namespaced topics at launch time — no recompilation needed.
- Run it with: `ros2 launch <pkg> turtlesim_mimic_launch.py`

For larger systems, `IncludeLaunchDescription` composes files. Arguments are passed as `.items()` on a dict:

```python
# Source: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        get_package_share_directory('my_robot') + '/launch/base.launch.py'
    ),
    launch_arguments={'use_sim_time': 'true'}.items()
)
```

---

## Common pitfalls

**1. Launch argument scope leaks across included files.**  
By default, launch arguments are globally scoped. If you include the same sub-launch file twice with different arguments, the second invocation inherits the first's resolved values. Wrap each `IncludeLaunchDescription` in a `GroupAction` to get isolated scope via implicit `PushLaunchConfigurations`/`PopLaunchConfigurations` calls. ([Issue #313](https://github.com/ros2/launch/issues/313))

**2. File naming: must end in `.launch.py`.**  
The `ros2 launch` CLI tab-completion and package scanning only recognise files matching `*.launch.py` (or `.launch.xml` / `.launch.yaml`). A file named `my_system.py` won't be found by `ros2 launch my_pkg my_system.py` — rename it `my_system.launch.py`.

**3. Default argument values silently drop in nested files.**  
When a parent file calls `IncludeLaunchDescription` without explicitly passing an argument, the child's `default_value` on `DeclareLaunchArgument` may not be evaluated. Always pass required arguments explicitly via `launch_arguments={}` rather than relying on defaults propagating from the included file. ([Issue #593](https://github.com/ros2/launch/issues/593))

---

## Further reading

- [Creating a launch file — ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Managing large projects with ROS 2 launch — Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
- [rosetta_launch — side-by-side Python/XML/YAML launch examples (MetroRobots/GitHub)](https://github.com/MetroRobots/rosetta_launch)

---

*2026-05-30 | ROS 2 version: Jazzy / Humble*
