# ROS2 Bag Files — Recording, Playback, and Programmatic Access

`ros2 bag` records topic data to disk and replays it deterministically — the single most important tool for reproducing bugs, validating algorithms offline, and sharing datasets between teams. As of ROS 2 Iron, **MCAP is the default storage backend**, replacing SQLite3 for most workflows.

---

## What it is

A bag is a **directory** (not a single file) containing a storage database and a `metadata.yaml` that describes every recorded topic, its type, serialization format, and QoS profile. The reader reconstructs the schema from metadata alone — no live nodes needed.

Two storage backends ship with ROS 2:

| Backend | File extension | When to use |
|---|---|---|
| MCAP (default since Iron) | `.mcap` | Sensor streams, large data, Foxglove Studio compatible |
| SQLite3 | `.db3` | Simple logs, tools that can query SQL directly |

Key design changes from ROS 1 bags:
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

# Start in a paused state (spacebar to unpause in the terminal)
ros2 bag play my_session --start-paused
```

**Keyboard controls during playback** (Jazzy+):

| Key | Action |
|-----|--------|
| Space | Pause / Resume |
| → / ← | Step forward one message |
| ↑ | Increase playback rate by 10% |
| ↓ | Decrease playback rate by 10% |

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

*(Source: [ros2_cookbook — bag.md, mikeferguson](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/bag.md))*

---

## Converting ROS 1 bags

The `rosbags` Python package converts `.bag` files (ROS 1) to a valid ROS 2 bag directory with zero ROS dependencies — no bridge, no active ROS installation:

```bash
pip3 install rosbags
rosbag-convert my_recording.bag
# Produces: my_recording/ (SQLite3, ROS 2 compatible)
```

Then migrate to MCAP with compression:

```bash
cat > to_mcap.yaml << 'EOF'
output_bags:
- uri: my_recording_mcap
  all: true
  storage_id: mcap
  compression_mode: file
  compression_format: zstd
EOF
ros2 bag convert -i my_recording -o to_mcap.yaml
```

*(Source: [ros2_cookbook — bag.md](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/bag.md))*

---

## C++ API — Writing from a Node

The `rosbag2_cpp::Writer` lets you record bag data programmatically. Receiving a `SerializedMessage` (not a typed message) avoids a redundant deserialize→serialize round-trip:

```cpp
// Source: docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html
// (Apache 2.0 — ros2/ros2_documentation)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Opens an MCAP bag at "my_bag/" — MCAP is the default storage format
    writer_->open("my_bag");

    // Receive the already-serialized message; avoids a redundant deserialize→serialize step
    auto callback = [this](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      rclcpp::Time time_stamp = this->now();
      // write() accepts: serialized msg, topic name, message type string, timestamp
      writer_->write(msg, "chatter", "std_msgs/msg/String", time_stamp);
    };

    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}
```

**CMakeLists.txt:**

```cmake
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp std_msgs)

install(TARGETS simple_bag_recorder DESTINATION lib/${PROJECT_NAME})
```

**package.xml:**

```xml
<depend>rclcpp</depend>
<depend>rosbag2_cpp</depend>
<depend>std_msgs</depend>
```

`writer_->open("my_bag")` uses `StorageOptions` defaults: MCAP format, no compression, no conversion. Pass a `rosbag2_storage::StorageOptions` struct to override format, compression, or MCAP preset profiles (`fastwrite` for high-throughput; `zstd_small` for compressed random access).

---

## C++ API — Reading from a Bag

`rosbag2_transport::ReaderWriterFactory::make_reader()` returns a `rosbag2_cpp::Reader` that auto-detects the storage format. The node below reads back a recorded topic and republishes it at a fixed rate:

```cpp
// Source: docs.ros.org/en/jazzy/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html
// (Apache 2.0 — ros2/ros2_documentation)
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
public:
  PlaybackNode(const std::string & bag_filename)
  : Node("playback_node")
  {
    publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);

    // Open the bag — ReaderWriterFactory detects MCAP vs SQLite3 automatically
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_filename;
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);

    // Step through one message per timer tick (100 ms → 10 Hz replay)
    timer_ = this->create_wall_timer(100ms,
      [this]() { return this->timer_callback(); });
  }

private:
  void timer_callback()
  {
    while (reader_->has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

      if (msg->topic_name != "/turtle1/pose") {
        continue;  // skip unrelated topics
      }

      // Deserialize the raw CDR bytes into a typed ROS message
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      auto ros_msg = std::make_shared<turtlesim::msg::Pose>();
      serialization_.deserialize_message(&serialized_msg, ros_msg.get());

      publisher_->publish(*ros_msg);
      break;  // one message per timer tick
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
  rclcpp::Serialization<turtlesim::msg::Pose> serialization_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_directory>" << std::endl;
    return 1;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
```

**CMakeLists.txt** (use `rosbag2_transport` for the reader factory, not `rosbag2_cpp` directly):

```cmake
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(turtlesim REQUIRED)   # replace with your message package

add_executable(simple_bag_reader src/simple_bag_reader.cpp)
ament_target_dependencies(simple_bag_reader rclcpp rosbag2_transport turtlesim)
install(TARGETS simple_bag_reader DESTINATION lib/${PROJECT_NAME})
```

**Deserialisation note:** The `rclcpp::Serialization<T>` object is stateless and thread-safe — create one per message type as a member variable, not per-callback, to avoid repeated ROSIDL type-support lookups.

---

## Python API — Writing from a Node

```python
# Source: docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html
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

`rosbag2_py.SequentialReader` reads messages in timestamp order:

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

    # Build a topic-name → message-type map from bag metadata
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Optional: filter to specific topics before iterating (much faster on large bags)
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/odom']))

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        # msg is now a fully typed Python ROS message object
        print(f'{timestamp} [{topic}]: {msg}')
```

**Seeking by timestamp** (nanoseconds since epoch):

```python
# Jump to 5 seconds into the bag without iterating from the start
bag_metadata = reader.get_metadata()
start_ns = bag_metadata.starting_time.nanoseconds_since_epoch
reader.seek(start_ns + 5_000_000_000)   # 5 s in nanoseconds

topic, data, timestamp = reader.read_next()
```

`set_filter()` and `seek()` can be interleaved at any point — the filter persists across seeks. Call `reader.reset_filter()` to remove it.

---

## Python API — Filter / Transform Pattern (SequentialReader + SequentialWriter)

The canonical way to remove, rewrite, or selectively copy messages across bags — avoids editing files in place and leaves the original untouched:

```python
# Source: github.com/mikeferguson/ros2_cookbook/blob/main/pages/bag.md (Apache 2.0)
# Example: strip odom→base_link TF from a bag, keep everything else

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri='input_bag', storage_id='mcap'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                output_serialization_format='cdr'))

writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri='output_bag', storage_id='mcap'),
    rosbag2_py.ConverterOptions('', ''))

# Register all input topics in the output bag first
topic_types = reader.get_all_topics_and_types()
tf_typename = None
for topic_type in topic_types:
    writer.create_topic(topic_type)          # mirror schema into output
    if topic_type.name == '/tf':
        tf_typename = topic_type.type

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    filter_out = False

    if topic == '/tf' and tf_typename:
        msg_type = get_message(tf_typename)
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom':
                filter_out = True   # drop this entire TF message
                break

    if not filter_out:
        writer.write(topic, data, timestamp)
```

The same pattern applies to any per-message transformation: edit the deserialized object, re-serialize it with `serialize_message()`, and write the modified bytes back. This is the correct approach for fixing bad timestamps, remapping topic names, or splitting a bag by time window.

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
`ros2 bag play --clock` publishes on `/clock`, but nodes that do *not* have `use_sim_time: true` run on wall time and can diverge from playback. This manifests as TF extrapolation errors or stale sensor data in callbacks. Audit every node in your playback graph and ensure sim-time settings are consistent:
```bash
ros2 bag play my_session --clock
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

**4. `--start-offset` skips latched/transient-local messages.**
When you play from a mid-bag offset, `TRANSIENT_LOCAL` messages (e.g. `/map`, `robot_description`) that were published before the offset are not re-emitted. Subscribers that join after those messages will never receive them. Work around this by playing the first few seconds at high speed (`--rate 10.0`) rather than skipping with `--start-offset`, or by pre-seeding the latched topic via a separate static publisher.

**5. Editing the extracted `.db3` or `.mcap` file directly.**
Neither the SQLite3 nor the MCAP file is designed for manual edits — the `metadata.yaml` checksums and offsets become stale. To filter, reorder, or transform bag content, use `rosbag2_py.SequentialReader` + `SequentialWriter` in a Python script (see the filter/transform pattern above), then use `ros2 bag convert` if format migration is also needed.

**6. Forgetting `create_topic()` before `write()` in the C++ writer.**
Unlike the Python `SequentialWriter`, `rosbag2_cpp::Writer::write()` with the topic-name-and-type overload registers the topic automatically on first write. However, if you use the lower-level `write(serialized_msg)` overload, `create_topic()` must be called first — omitting it throws a `std::runtime_error` at the first write.

**7. `ros2 bag convert` writing zero messages on Jazzy.**
A known issue ([rosbag2#1660](https://github.com/ros2/rosbag2/issues/1660)) causes `ros2 bag convert` to silently produce an empty output bag in some Jazzy configurations. Verify output with `ros2 bag info <output>` immediately after conversion. If message counts are 0, use the `rosbag2_py` Reader+Writer script above instead of `ros2 bag convert` as a workaround.

---

## Further reading

- [Recording and Playing Back Data — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) — official CLI tutorial covering basic record, info, and play
- [Recording a Bag from Your Own Node (Python) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html) — SequentialWriter from inside a Python node with TopicMetadata
- [Recording a Bag from Your Own Node (C++) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html) — rosbag2_cpp::Writer with SerializedMessage subscription pattern
- [Reading from a Bag File (C++) — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html) — ReaderWriterFactory, has_next/read_next loop, rclcpp::Serialization deserialization
- [Overriding QoS Policies for Recording and Playback — ROS 2 Jazzy](https://docs.ros.org/en/jazzy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html) — QoS override YAML format and use cases for BEST_EFFORT topics
- [ros2_cookbook — bag.md (mikeferguson, GitHub)](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/bag.md) — ROS 1 conversion, compression, and the TF-filtering Reader+Writer pattern used in this article
- [MCAP Getting Started — ROS 2 (mcap.dev)](https://mcap.dev/guides/python/ros2) — MCAP format internals, Foxglove Studio integration, and Python reader libraries
- [rosbag2_storage_mcap — ROS 2 Jazzy package docs](https://docs.ros.org/en/jazzy/p/rosbag2_storage_mcap/) — MCAP storage plugin, preset profiles (fastwrite, zstd_small), and configuration options

---

*2026-08-22 | ROS2 version: Jazzy (MCAP default) / Humble (SQLite3 default)*
