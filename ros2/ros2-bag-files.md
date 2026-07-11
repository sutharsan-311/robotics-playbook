# ROS2 Bag Files — Recording, Playback, and Programmatic Access

`ros2 bag` records topic data to disk and replays it deterministically — the single most important tool for reproducing bugs, validating algorithms offline, and sharing datasets between teams. As of ROS 2 Iron, **MCAP is the default storage backend**, replacing SQLite3 for most workflows.

---

## What it is

A bag is a **directory** (not a single file) containing a storage database and a `metadata.yaml` that describes every recorded topic, its type, serialization format, and QoS profile. The reader reconstructs the schema from metadata alone — no live nodes needed.

Two storage backends ship with ROS 2:

| Backend | File extension | When to use |
|---|---|---|
| MCAP (default since Iron) | `.mcap` | Sensor streams, large data, Foxglove/Studio compatible |
| SQLite3 | `.db3` | Simple logs, tools that can query SQL directly |

Key design changes from ROS1 bags:
- The bag is a **directory**, not a single `.bag` file.
- QoS profiles are stored per topic and re-applied on playback — a common source of silent zero-message topics (see pitfalls).
- Storage backends are runtime-swappable via `--storage`.
- Playback exposes a `/clock` topic, making sim-time replay first-class.

---

## CLI — Recording

```bash
# Record specific topics (MCAP by default on Jazzy/Iron+)
ros2 bag record /scan /odom /tf

# Explicit storage backend
ros2 bag record -s mcap /scan /odom
ros2 bag record -s sqlite3 /scan /odom

# Named output directory
ros2 bag record -o my_session /scan /odom

# Record everything
ros2 bag record -a

# Record all topics matching a regex (e.g. all camera topics)
ros2 bag record --regex "/camera.*"

# Exclude topics matching a regex (record all except TF)
ros2 bag record -a --exclude "/tf.*"

# Split the bag every 1 GB or every 60 seconds (whichever comes first)
ros2 bag record -a --max-bag-size 1073741824 --max-bag-duration 60

# Record with file-level zstd compression (~75% size reduction for sensor bags)
ros2 bag record -a --compression-mode file --compression-format zstd

# Start recording in paused state — useful to gate recording from a launch file
ros2 bag record -a --start-paused
```

`--max-bag-size` and `--max-bag-duration` split the bag into rolling files, which prevents a single corrupt write from destroying the full session and keeps individual files under network-share size limits.

---

## CLI — Inspect and Playback

```bash
# Inspect a recorded bag — per-topic counts, duration, QoS, storage id
ros2 bag info my_session

# Play at normal speed
ros2 bag play my_session

# Play and publish /clock (required for nodes using use_sim_time: true)
ros2 bag play my_session --clock

# Play at 0.5× speed, looping
ros2 bag play my_session --rate 0.5 --loop

# Skip to 30 s into the bag before playing
ros2 bag play my_session --start-offset 30.0

# Play only the first 60 seconds of the bag
ros2 bag play my_session --duration 60.0

# Play only selected topics
ros2 bag play my_session --topics /scan /odom

# Play topics matching a regex
ros2 bag play my_session --regex "/camera.*"

# Start in a paused state (space to unpause)
ros2 bag play my_session --start-paused
```

`ros2 bag info` is the first command to run when debugging a playback problem — it shows per-topic message counts and the stored QoS profile. Zero message counts almost always indicate a QoS mismatch at record time.

---

## CLI — Convert and Repack

`ros2 bag convert` rewrites one or more input bags into one or more output bags with new storage settings — the standard way to migrate formats, apply compression, or merge bag sessions:

```bash
# Convert SQLite3 → MCAP with zstd compression
# 1. Write a conversion spec:
cat > output_format.yaml << 'EOF'
output_bags:
- uri: my_session_compressed
  all: true
  storage_id: mcap
  compression_mode: file
  compression_format: zstd
EOF

# 2. Run the conversion
ros2 bag convert -i my_session -o output_format.yaml

# Merge two bag sessions into one output
cat > merge_spec.yaml << 'EOF'
output_bags:
- uri: merged_session
  all: true
EOF
ros2 bag convert -i session_a -i session_b -o merge_spec.yaml
```

Typical compression results for sensor-heavy bags (LiDAR + camera): ~65–75% size reduction with `zstd` and negligible decompression overhead during playback.

---

## Python API — Writing from a Node

The `rosbag2_py.SequentialWriter` lets you record bag data programmatically from any node. The following example is taken directly from the [official Jazzy docs](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html):

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
            storage_id='mcap')           # 'sqlite3' also valid
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name='/chatter',
            type='std_msgs/msg/String',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)  # must call before first write()

        self.subscription = self.create_subscription(
            String, '/chatter', self.topic_callback, 10)

    def topic_callback(self, msg):
        self.writer.write(
            '/chatter',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)   # nanoseconds since epoch
```

`create_topic()` must be called once per topic before the first `write()` for that topic. `serialize_message()` converts the Python message object to CDR bytes. Pass nanoseconds from the node's clock — if `use_sim_time` is set, `get_clock().now()` returns sim time automatically.

---

## Python API — Reading from a Bag

`rosbag2_py.SequentialReader` reads messages back in timestamp order. This pattern is from `ros2_cookbook` (mikeferguson, GitHub) and the `rosbag2_py` test suite:

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_bag(bag_path: str, storage_id: str = 'mcap'):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'))

    # Build a topic-name → message-type map
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Optional: filter to specific topics before iterating
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/odom']))

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        # msg is now a fully typed Python ROS message object
        print(f'{timestamp} [{topic}]: {msg}')
```

**Seeking by timestamp** (nanoseconds since epoch) lets you jump into the middle of a long bag without iterating from the start:

```python
# Jump to 5 seconds into the bag
bag_metadata = reader.get_metadata()
start_ns = bag_metadata.starting_time.nanoseconds_since_epoch
reader.seek(start_ns + 5_000_000_000)   # 5 s in nanoseconds

# read_next() will return messages from that point onward
topic, data, timestamp = reader.read_next()
```

`set_filter()` and `seek()` can be interleaved at any point — the filter persists across seeks. Call `reader.reset_filter()` to remove it.

---

## Common pitfalls

**1. QoS mismatch silently records zero messages.**  
When a publisher uses `BEST_EFFORT` reliability, a `RELIABLE` subscriber (rosbag2's default) is incompatible under DDS — no messages are delivered and no error is raised. Check `ros2 bag info` after recording: if any topic count is 0, override the QoS:
```bash
ros2 bag record -a --qos-profile-overrides-path qos_override.yaml
```
The override YAML format is documented in the [rosbag2 QoS guide](https://docs.ros.org/en/jazzy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html).

**2. Recording with `--use-sim-time` before `/clock` is publishing causes epoch-zero timestamps.**  
If rosbag2 starts recording before the `/clock` topic emits its first message, all bag timestamps are written as `0` (Unix epoch). The bag becomes unplayable or produces wildly wrong timing. Always ensure the clock source (a playing bag, Gazebo, etc.) is live before starting a new recording session. A fixed sleep in launch files is not reliable — use a `ReadyCondition` or lifecycle node to gate the recorder.

**3. Nodes on wall time outrun or lag behind bag playback.**  
`ros2 bag play --clock` publishes on `/clock`, but nodes that do *not* have `use_sim_time: true` run on wall time and can diverge from playback. This manifests as TF extrapolation errors or stale sensor data in callbacks. Audit every node in your playback graph and ensure sim-time settings are consistent. Run:
```bash
ros2 bag play my_session --clock
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

**4. `--start-offset` skips latched/transient-local messages.**  
When you play from a mid-bag offset, `TRANSIENT_LOCAL` messages (e.g. `/map`, `robot_description`) that were published before the offset are not re-emitted. Subscribers that join after those messages will never receive them. Work around this by playing the first few seconds at high speed (`--rate 10.0`) rather than skipping with `--start-offset`, or by pre-seeding the latched topic via a separate static publisher.

**5. Editing the extracted `.db3` or `.mcap` file directly.**  
Neither the SQLite3 nor the MCAP file is designed for manual edits — the `metadata.yaml` checksums and offsets become stale. To filter, reorder, or transform bag content, use `rosbag2_py.SequentialReader` + `SequentialWriter` in a Python script (see the ros2_cookbook filter-TF example), then use `ros2 bag convert` if format migration is also needed.

---

## Further reading

- [Recording and Playing Back Data — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — official CLI tutorial
- [Recording a Bag from Your Own Node (Python) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html) — SequentialWriter from inside a node
- [Reading from a Bag File (C++) — ROS 2 Rolling](https://docs.ros.org/en/rolling/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html) — C++ SequentialReader API with StorageOptions
- [Overriding QoS Policies for Recording and Playback — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html) — QoS override YAML format and use cases
- [ros2_cookbook — bag.md (mikeferguson, GitHub)](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/bag.md) — real-world patterns: ROS1 migration, compression conversion, filtering TF from a bag with SequentialReader + SequentialWriter
- [MCAP Getting Started — ROS 2 (mcap.dev)](https://mcap.dev/guides/getting-started/ros-2) — MCAP format internals, Foxglove Studio integration, and Python/C++ reader libraries

---

*2026-07-11 | ROS2 version: Jazzy (MCAP default) / Humble (SQLite3 default)*
