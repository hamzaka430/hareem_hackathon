---
sidebar_position: 3
---

# ROS2 Nodes and Topics

This section provides hands-on guidance for creating ROS2 nodes and working with the topic-based communication system.

## Creating a ROS2 Package

### Python Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg --dependencies rclpy std_msgs geometry_msgs
```

Generated structure:
```
my_robot_pkg/
├── my_robot_pkg/
│   └── __init__.py
├── resource/
│   └── my_robot_pkg
├── test/
├── package.xml
├── setup.cfg
└── setup.py
```

### C++ Package

```bash
ros2 pkg create --build-type ament_cmake my_robot_cpp --dependencies rclcpp std_msgs
```

## Publisher Node

### Python Publisher

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(
            Twist,           # Message type
            'cmd_vel',       # Topic name
            10               # Queue size
        )
        
        # Create timer (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.1)
        
        self.get_logger().info('Velocity Publisher started')
    
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_speed').value
        msg.angular.z = self.get_parameter('angular_speed').value
        
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Publisher

```cpp
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&VelocityPublisher::publish_velocity, this));
        
        this->declare_parameter("linear_speed", 0.5);
        this->declare_parameter("angular_speed", 0.1);
        
        RCLCPP_INFO(this->get_logger(), "Velocity Publisher started");
    }

private:
    void publish_velocity() {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = this->get_parameter("linear_speed").as_double();
        msg.angular.z = this->get_parameter("angular_speed").as_double();
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Subscriber Node

### Python Subscriber

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )
        
        self.get_logger().info('Velocity Subscriber started')
    
    def velocity_callback(self, msg):
        self.get_logger().info(
            f'Received: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Combined Publisher/Subscriber

A single node can both publish and subscribe:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.min_distance = 0.5  # meters
        
    def scan_callback(self, msg):
        # Get minimum distance in front
        front_ranges = msg.ranges[len(msg.ranges)//2-30:len(msg.ranges)//2+30]
        min_front = min(r for r in front_ranges if r > msg.range_min)
        
        cmd = Twist()
        
        if min_front < self.min_distance:
            # Obstacle detected - turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn(f'Obstacle at {min_front:.2f}m - turning')
        else:
            # Clear path - go forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Sensor Messages

### IMU Data

```python
from sensor_msgs.msg import Imu

def imu_callback(self, msg):
    # Orientation (quaternion)
    orientation = msg.orientation
    
    # Angular velocity (rad/s)
    angular_vel = msg.angular_velocity
    
    # Linear acceleration (m/s²)
    linear_acc = msg.linear_acceleration
```

### Camera Image

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Process with OpenCV...
```

### Point Cloud

```python
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

def pointcloud_callback(self, msg):
    # Iterate through points
    for point in pc2.read_points(msg, field_names=['x', 'y', 'z']):
        x, y, z = point
        # Process point...
```

## Topic Tools

### Introspection

```bash
# List all topics
ros2 topic list

# List topics with types
ros2 topic list -t

# Show topic info
ros2 topic info /cmd_vel

# Show message type
ros2 topic type /cmd_vel

# Show message definition
ros2 interface show geometry_msgs/msg/Twist
```

### Debugging

```bash
# Echo messages
ros2 topic echo /cmd_vel

# Echo with type
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist

# Measure frequency
ros2 topic hz /cmd_vel

# Measure bandwidth
ros2 topic bw /cmd_vel
```

### Testing

```bash
# Publish test message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"

# Publish once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Publish at rate
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
```

## Best Practices

### Naming Conventions

```
/namespace/node_name          # Node names
/namespace/topic_name         # Topic names
/namespace/service_name       # Service names
/namespace/action_name        # Action names
```

### Topic Organization

```
/robot/
├── sensors/
│   ├── imu/data
│   ├── camera/image_raw
│   └── lidar/scan
├── actuators/
│   ├── cmd_vel
│   └── joint_commands
└── state/
    ├── odometry
    └── joint_states
```

### Message Design

- Use existing messages when possible
- Keep messages small and focused
- Include timestamps (`header.stamp`)
- Use appropriate data types

## Key Takeaways

1. **Publishers** send messages to topics
2. **Subscribers** receive messages from topics
3. **One node can have** multiple publishers and subscribers
4. **Use standard messages** from `std_msgs`, `geometry_msgs`, `sensor_msgs`
5. **Topic tools** help with debugging and testing
6. **Organize topics** with namespaces

---

*Next Chapter: [Computer Vision for Robots](../chapter-05/vision-fundamentals.md) →*
