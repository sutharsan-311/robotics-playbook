# ROS2 Bag Files — Recording and Playback

`ros2 bag` lets you record topic data to disk and replay it deterministically — the single most important tool for reproducing bugs, validating algorithms offline, and sharing datasets between teams.

## What it is

A bag file is a time-stamped, topic-keyed database of serialized ROS2 messages. Under the hood, rosbag2 uses a pluggable storage backend (SQLite3 by default, MCAP increasingly common) and stores a `metadata.yaml` alongside the data file so the reader knows the schema without needing live nodes.

Key design differences from ROS1 bags:
- **No single `.bag` file** — a bag is a *directory* containing the database and metadata.
- **Storage backends are swappable** — SQLite3 ships by default; MCAP (`ros-humble-rosbag2-storage-mcap`) is preferred for large sensor streams.
- **QoS profiles are recorded** per topic and re-applied on playback, which is both a feature and a common source of surprises (see pitfalls).

## How it works

### CLI — record, inspect, replay

```bash
# Record a single topic
ros2 bag record /turtle1/cmd_vel

# Record multiple topics with a named output directory
ros2 bag record -o my_session /turtle1/cmd_vel /turtle1/pose

# Record every topic on the system
ros2 bag record -a

# Inspect a recorded bag
ros2 bag info my_session

# Replay at normal speed
ros2 bag play my_session

# Replay at 0.5× speed, looping
ros2 bag play my_session --rate 0.5 --loop

# Replay only specific topics
ros2 bag play my_session --topics /turtle1/cmd_vel
```

`ros2 bag info` reports per-topic message counts, duration, and the QoS profile stored for each topic — always check this before debugging a playback issue.

### Python API — recording from inside a node

The `rosbag2_py` package exposes a `SequentialWriter` that lets you write bag data programmatically from any node. The following example is taken directly from the [official Humble docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html):

```python
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String

import rosbag2_py

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py.StorageOptions(
            uri='my_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name='chatter',
            type='std_msgs/msg/String',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.subscription = self.create_subscription(
            String, 'chatter', self.topic_callback, 10)

    def topic_callback(self, msg):
        self.writer.write(
            'chatter',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)
```

The key points: `serialize_message()` converts the Python msg object to CDR bytes; `get_clock().now().nanoseconds` stamps the write with the node's clock (use sim time if `use_sim_time` is set). `create_topic()` must be called before the first `write()` for that topic.

## Common pitfalls

**1. QoS mismatch drops messages silently.**  
When you record with `ros2 bag record -a`, rosbag2 subscribes using the QoS profile it discovers on each topic. If a publisher uses `BEST_EFFORT` reliability, rosbag2 may subscribe as `RELIABLE`, causing zero messages to be recorded — no error, just an empty topic in the bag. Fix: check `ros2 bag info` after recording and, if counts are 0, override explicitly:
```bash
ros2 bag record -a --qos-profile-overrides-path qos_override.yaml
```
The override YAML format is documented in the [rosbag2 QoS guide](https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html).

**2. `--use-sim-time` during record causes epoch-zero timestamps.**  
If you start recording before the `/clock` topic publishes its first message, the bag's internal timestamps are written as `0` (Jan 1, 1970). The bag becomes unplayable or produces wildly wrong timing. Always ensure your clock source (a bag playing back, Gazebo, etc.) is publishing `/clock` *before* starting a new recording session. A one-second sleep in launch files is not reliable — use a topic-wait or lifecycle node to gate the recorder.

**3. Playback doesn't account for clock — subscribers miss messages.**  
By default, `ros2 bag play` publishes on `/clock` so nodes using sim time step in sync. But nodes that do *not* have `use_sim_time: true` run on wall clock and can outrun or lag behind playback. This manifests as TF extrapolation errors or stale sensor data in callbacks. Audit all nodes in your playback graph and ensure their sim-time setting is consistent.

## Further reading

- [Recording and Playing Back Data — ROS 2 Humble Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
- [Recording a Bag from Your Own Node (Python) — ROS 2 Humble Docs](https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html)
- [rosbag2: Overriding QoS Policies — ROS 2 How-To Guides](https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html)

---

*2026-06-30 | ROS2 versions: Jazzy / Humble*
