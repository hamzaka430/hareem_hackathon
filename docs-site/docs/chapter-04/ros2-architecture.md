---
sidebar_position: 2
---

# ROS2 Architecture

Understanding ROS2's architecture helps you build robust robotic systems. This section dives into the layers and components that make ROS2 work.

## Layered Architecture

```
┌─────────────────────────────────────────────────┐
│              Your Application                   │
│         (Nodes, Packages, Launch files)         │
├─────────────────────────────────────────────────┤
│              ROS2 Client Libraries              │
│           (rclpy, rclcpp, rclnodejs)           │
├─────────────────────────────────────────────────┤
│                    RCL                          │
│          (ROS Client Library - C)               │
├─────────────────────────────────────────────────┤
│                    RMW                          │
│        (ROS Middleware Interface)               │
├─────────────────────────────────────────────────┤
│              DDS Implementation                 │
│     (Fast-DDS, Cyclone DDS, RTI Connext)       │
├─────────────────────────────────────────────────┤
│              Operating System                   │
│          (Linux, Windows, macOS)                │
└─────────────────────────────────────────────────┘
```

## DDS: The Foundation

### What is DDS?

Data Distribution Service - A standard for real-time publish/subscribe communication.

**Key Features**:
- **Discovery**: Automatic node discovery (no central master!)
- **QoS**: Quality of Service policies
- **Real-time**: Deterministic communication
- **Scalable**: Works from embedded to distributed systems

### Available DDS Implementations

| Implementation | Organization | License |
|---------------|--------------|---------|
| Fast-DDS | eProsima | Apache 2.0 |
| Cyclone DDS | Eclipse | EPL 2.0 |
| RTI Connext | RTI | Commercial |

**Default**: Fast-DDS (as of Humble)

### Switching DDS

```bash
# Set environment variable
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Or for RTI Connext
export RMW_IMPLEMENTATION=rmw_connextdds
```

## Quality of Service (QoS)

QoS policies control how messages are handled.

### Key QoS Policies

| Policy | Options | Purpose |
|--------|---------|---------|
| Reliability | RELIABLE / BEST_EFFORT | Message delivery guarantee |
| Durability | VOLATILE / TRANSIENT_LOCAL | Late-joiner message access |
| History | KEEP_LAST(n) / KEEP_ALL | Message buffering |
| Depth | Integer | Buffer size |
| Deadline | Duration | Message timing |

### Common QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Sensor data (high frequency, okay to drop)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)

# Commands (must be delivered)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Parameters (must be delivered, need history)
parameter_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)
```

### QoS Compatibility

Publishers and subscribers must have compatible QoS:

```
Publisher          Subscriber         Compatible?
─────────────────────────────────────────────────
RELIABLE    →     RELIABLE           ✓
RELIABLE    →     BEST_EFFORT        ✓
BEST_EFFORT →     RELIABLE           ✗
BEST_EFFORT →     BEST_EFFORT        ✓
```

## Executors

Executors manage how callbacks are processed.

### Single-Threaded Executor

```python
rclpy.init()
node = MyNode()
rclpy.spin(node)  # Uses SingleThreadedExecutor
```

All callbacks run sequentially in one thread.

### Multi-Threaded Executor

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

Callbacks can run in parallel (watch for race conditions!).

### Callback Groups

Control callback execution:

```python
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # These callbacks can run simultaneously
        self.reentrant_group = ReentrantCallbackGroup()
        
        # These callbacks are mutually exclusive
        self.exclusive_group = MutuallyExclusiveCallbackGroup()
        
        self.sub1 = self.create_subscription(
            String, 'topic1', self.callback1,
            10, callback_group=self.reentrant_group)
```

## Message Types

### Standard Messages

Located in packages like `std_msgs`, `geometry_msgs`, `sensor_msgs`:

```bash
# List message types
ros2 interface list

# Show message definition
ros2 interface show geometry_msgs/msg/Twist
```

**geometry_msgs/msg/Twist**:
```
Vector3 linear
    float64 x
    float64 y
    float64 z
Vector3 angular
    float64 x
    float64 y
    float64 z
```

### Custom Messages

Define in `msg/` folder:

**MyMessage.msg**:
```
uint32 id
string name
float64[] values
geometry_msgs/Pose pose
```

### Common Message Packages

| Package | Contents |
|---------|----------|
| std_msgs | Basic types (String, Int, Float, Bool) |
| geometry_msgs | Poses, transforms, velocities |
| sensor_msgs | Camera, IMU, laser scans, point clouds |
| nav_msgs | Odometry, maps, paths |
| visualization_msgs | Markers for RViz |

## Transforms (TF2)

TF2 manages coordinate frame relationships.

### The TF Tree

```
       world
          │
          │
       base_link
        /     \
       /       \
   camera    lidar
```

### Broadcasting Transforms

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
    
    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 1.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
```

### Listening to Transforms

```python
from tf2_ros import Buffer, TransformListener

class TransformSubscriber(Node):
    def __init__(self):
        super().__init__('transform_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def get_transform(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'world', 'camera',
                rclpy.time.Time())
            return t
        except Exception as e:
            self.get_logger().warn(f'Transform error: {e}')
            return None
```

## Launch System

Launch files start and configure multiple nodes.

### Python Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='custom_name',
            parameters=[{'param1': 'value1'}],
            remappings=[('/old_topic', '/new_topic')]
        ),
        Node(
            package='other_package',
            executable='other_node'
        )
    ])
```

### Running Launch Files

```bash
ros2 launch my_package my_launch.py
```

## Key Takeaways

1. **DDS** provides decentralized, real-time communication
2. **QoS policies** control message reliability and delivery
3. **Executors** manage callback execution (single vs multi-threaded)
4. **TF2** tracks coordinate frame relationships
5. **Launch files** orchestrate multi-node systems

---

*Next: [ROS2 Nodes and Topics](./ros2-nodes-topics.md) →*
