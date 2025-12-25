---
sidebar_position: 1
---

# Introduction to ROS2

ROS2 (Robot Operating System 2) is the industry-standard middleware for robotics development. This chapter introduces ROS2's architecture and core concepts.

## What is ROS2?

ROS2 is **not** an operating system. It's a:
- **Middleware**: Communication between processes
- **Framework**: Standard patterns for robotics software
- **Ecosystem**: Libraries, tools, and community packages

### ROS1 vs ROS2

| Aspect | ROS1 | ROS2 |
|--------|------|------|
| Communication | Custom (TCPROS) | DDS standard |
| Real-time | Limited | Supported |
| Multi-robot | Difficult | Native |
| Security | None | Built-in |
| Platforms | Linux only | Linux, Windows, macOS |
| Support | Maintenance | Active development |

**ROS2 is the future** - Use it for all new projects.

## Core Concepts

### Nodes

A node is a single-purpose process:

```
┌────────────────────────────────────────────┐
│              Robot System                   │
│                                            │
│  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐  │
│  │Camera│  │Lidar │  │Planner│  │Motor │  │
│  │ Node │  │ Node │  │ Node  │  │ Node │  │
│  └──────┘  └──────┘  └──────┘  └──────┘  │
│                                            │
└────────────────────────────────────────────┘
```

**Design principle**: One node = one task

### Topics

Asynchronous publish/subscribe communication:

```
Publisher                           Subscriber
┌────────┐    /camera/image        ┌────────┐
│ Camera │  ──────────────────────▶│ Vision │
│  Node  │       Topic             │  Node  │
└────────┘                         └────────┘
```

- **Publisher**: Sends messages
- **Subscriber**: Receives messages
- **Topic**: Named channel (e.g., `/cmd_vel`)
- **Message**: Data structure

### Services

Synchronous request/response:

```
Client                              Server
┌────────┐    /set_pose            ┌────────┐
│Planner │  ──────Request─────────▶│ Robot  │
│  Node  │  ◀─────Response─────────│ Driver │
└────────┘       Service           └────────┘
```

- **Client**: Sends request, waits for response
- **Server**: Processes request, returns response
- **Service**: Named endpoint

### Actions

Long-running tasks with feedback:

```
Action Client                    Action Server
┌──────────┐   /navigate         ┌──────────┐
│ Planning │  ────Goal──────────▶│Navigation│
│   Node   │  ◀──Feedback────────│  Server  │
│          │  ◀──Result──────────│          │
└──────────┘                     └──────────┘
```

Components:
- **Goal**: What to accomplish
- **Feedback**: Progress updates
- **Result**: Final outcome
- **Cancellation**: Can be cancelled

### Parameters

Runtime configuration:

```python
# Setting a parameter
self.declare_parameter('max_speed', 1.0)

# Getting a parameter
max_speed = self.get_parameter('max_speed').value
```

## Installation

### Ubuntu (Recommended)

```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (LTS)
sudo apt update
sudo apt install ros-humble-desktop

# Source setup
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Windows

```powershell
# Using chocolatey
choco install ros-humble-desktop
```

Or download installer from ROS2 releases page.

## First ROS2 Program

### Python Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'greeting', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, World! #{self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Node

```cpp
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloPublisher : public rclcpp::Node {
public:
    HelloPublisher() : Node("hello_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("greeting", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HelloPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello, World! #" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HelloPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Essential ROS2 Commands

### Node Commands

```bash
# List running nodes
ros2 node list

# Get info about a node
ros2 node info /my_node
```

### Topic Commands

```bash
# List topics
ros2 topic list

# Show topic info
ros2 topic info /cmd_vel

# Echo topic messages
ros2 topic echo /cmd_vel

# Publish a message
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```

### Service Commands

```bash
# List services
ros2 service list

# Call a service
ros2 service call /set_pose geometry_msgs/srv/SetPose "{pose: {position: {x: 1.0}}}"
```

### Package Commands

```bash
# List packages
ros2 pkg list

# Create a new package
ros2 pkg create --build-type ament_python my_package

# Build packages
colcon build
```

## Workspace Structure

```
ros2_ws/
├── src/
│   ├── my_package/
│   │   ├── my_package/
│   │   │   ├── __init__.py
│   │   │   └── my_node.py
│   │   ├── resource/
│   │   ├── test/
│   │   ├── package.xml
│   │   └── setup.py
│   └── other_package/
├── build/        # Build artifacts
├── install/      # Installed packages
└── log/          # Build logs
```

## Key Takeaways

1. **ROS2 is middleware** for robotics communication and development
2. **Nodes** are single-purpose processes that communicate
3. **Topics** provide async pub/sub communication
4. **Services** provide sync request/response
5. **Actions** handle long-running tasks with feedback
6. **Use colcon** to build and manage workspaces

---

*Next: [ROS2 Architecture](./ros2-architecture.md) →*
