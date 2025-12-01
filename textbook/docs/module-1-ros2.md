# Module 1: ROS 2 – Robot Operating System

## Chapter 1: Introduction to ROS 2

### 1.1 What is ROS 2?
Robot Operating System 2 (ROS 2) is a middleware framework used to build advanced robotics applications.  
It provides tools and communication layers that allow different robot components to talk to each other.

### 1.2 Key Concepts
- **Nodes** – Independent processes that perform tasks  
- **Topics** – Communication channels for data exchange  
- **Services** – Request/response communication  
- **Actions** – Long-running tasks with feedback  

### 1.3 ROS 2 Architecture Diagram
```
+------------------+        +------------------+        +------------------+
|     Node 1       |        |      Topic       |        |     Node 2       |
|  (Sensor Input)  | -----> |     /camera      | -----> |   (AI Module)    |
+------------------+        +------------------+        +------------------+
```


---

## 1.4 Installing ROS 2 on Ubuntu 22.04

```bash
# Update system and set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Configure ROS 2 sources
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop
```

---

# 1.5 ROS 2 Python Node Example  
**Simple working robot controller for Pakistani students**

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Node Example
For Pakistani Students - Complete working code
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleRobot(Node):
    """A simple robot controller node"""
    
    def __init__(self):
        super().__init__('pakistani_robot')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.get_logger().info('✅ Robot node initialized successfully!')
        
    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.1
        
        self.velocity_publisher.publish(msg)
        self.get_logger().info(
            f'📤 Published velocity: linear={msg.linear.x}, angular={msg.angular.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    robot = SimpleRobot()

    print("\n==============================================")
    print("🤖 PAKISTANI ROBOT CONTROLLER")
    print("Node Name: pakistani_robot")
    print("Publishing to: /cmd_vel")
    print("Press Ctrl+C to exit")
    print("==============================================\n")

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        print("\n🛑 Robot stopped by user")
    finally:
        robot.destroy_node()
        rclpy.shutdown()
        print("✅ ROS 2 shutdown complete")

if __name__ == '__main__':
    main()
```

---

# 1.6 Temperature Sensor Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensor(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, '/temperature', 10)
        self.timer = self.create_timer(2.0, self.publish_temperature)
        
    def publish_temperature(self):
        temp = random.uniform(20.0, 40.0)
        msg = Float32()
        msg.data = temp
        self.publisher.publish(msg)
        self.get_logger().info(f'Temperature: {temp:.1f}°C')
```

---

# 1.7 Quiz Section

<details>
<summary><strong>Q1: What is the difference between ROS 1 and ROS 2?</strong></summary>

**A:** ROS 2 includes real-time support, security, DDS-based communication, and multi-platform compatibility.

</details>

<details>
<summary><strong>Q2: How do nodes communicate in ROS 2?</strong></summary>

**A:** Nodes communicate using **topics**, **services**, and **actions** depending on the requirement.

</details>

---

### 🎉 End of Module 1  
Ready for **Module 2: Gazebo Simulation**?  
Say: **Generate Module 2** 🚀
