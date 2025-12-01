# Module 2 — Gazebo Simulation  
### **High-Fidelity Physics for Robotics**

---

# 📘 **Chapter 2: Physics Simulation for Robotics**

Robots must work in the real world — where physics matters.

Before building an expensive robot, engineers test everything in **simulation**:

- movement  
- collisions  
- sensors  
- navigation  
- AI models  
- robot control algorithms  

Gazebo is one of the most widely used simulators in:

- Research labs  
- Universities  
- ROS-powered robots  
- Autonomous vehicle development  
- Drone simulation  
- Humanoid robot training  

This module will walk you through:

1. Installing Gazebo  
2. Creating a world  
3. Importing robots using URDF  
4. Running physics simulation  
5. Using ROS 2 to control robots  
6. Creating environments  
7. Adding sensors  
8. Running practical exercises  

---

# ⭐ **2.1 Introduction to Gazebo**

Gazebo is a **3D physics simulator** that works with ROS 2.

### It provides:

| Feature | Description |
|--------|-------------|
| 🧪 **Physics** | Gravity, friction, collisions, rigid bodies |
| 🎥 **Rendering** | Real-time 3D graphics |
| 📡 **Sensors** | Cameras, LiDAR, IMU, GPS |
| 🤖 **Robot Models** | URDF & SDF robot descriptions |
| 🚀 **Plugins** | ROS 2 integration through Gazebo plugins |
| 🌍 **Environment Building** | Rooms, buildings, roads, worlds |

---

# ⭐ **2.2 Installing Gazebo + ROS 2**

**For Ubuntu 22.04 + ROS Humble**

```bash
sudo apt update

# Install Gazebo
sudo apt install gazebo11 -y

# ROS 2 Integration Packages
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-plugins -y
sudo apt install ros-humble-gazebo-ros2-control -y
```

Verify installation:

```bash
gazebo
```

If the 3D window opens — you're ready 🎉

---

# ⭐ **2.3 Understanding Gazebo Physics**

Gazebo uses real physics engines to simulate motion:

### ⚙ Physics Engines Supported:

- **ODE** (Default engine)
- **Bullet**
- **SimBody**
- **DART**

### What physics is simulated?

| Physics | Example |
|--------|---------|
| Gravity | Object falling off a shelf |
| Collision detection | Robot hitting a wall |
| Inertia | Heavy vs light object movement |
| Torque | Joint motors |
| Friction | Robot wheels slipping |
| Air resistance | Drone simulation |

This allows you to test:

- Walking humanoids  
- Wheeled robots  
- Manipulators  
- Drones & UAVs  
- Underwater robots  

---

# ⭐ **2.4 Your First Gazebo World**

Let’s create a simple world.

Create file:  
`my_world.sdf`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">

  <world name="pakistani_world">

    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

  </world>

</sdf>
```

Run it:

```bash
gazebo my_world.sdf
```

---

# ⭐ **2.5 Launching Gazebo from ROS 2**

Create a launch file:

`launch/gazebo_launch.py`

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
    ])
```

Run:

```bash
ros2 launch my_robot gazebo_launch.py
```

---

# ⭐ **2.6 Creating a Custom Room (10×10 meters)**  
### With walls + table + robot

This is your practical exercise.

---

## 📌 **WORLD STRUCTURE**
You will build:

```
   +------------------------+
   |                        |
   |    ( Table Here )      |
   |          ███           |
   |                        |
   +------------------------+
```

- 10×10 meter room  
- 4 walls  
- A table in the center  
- A robot spawns inside  
- You control robot to move around the table  

---

## 🔨 **2.6.1 Walls of the Room**

Add this inside `<world>`:

```xml
<!-- Four walls -->
<model name="wall_north">
  <pose>0 5 1.25 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>10 0.2 2.5</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>10 0.2 2.5</size></box></geometry>
    </visual>
  </link>
</model>
```

Repeat for:

- wall_south  
- wall_east  
- wall_west  

Changing position accordingly.

---

## 🔨 **2.6.2 Table in the Center**

```xml
<model name="table">
  <pose>0 0 0.75 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>1 1 1.5</size></box>
      </geometry>
    </visual>
  </link>
</model>
```

---

# ⭐ **2.7 Adding a Robot (URDF)**

Create `robot.urdf`:

```xml
<robot name="simple_bot">
  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.2"/></geometry>
      <material><color rgba="0 0.6 1 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.4 0.3 0.2"/></geometry>
    </collision>
  </link>
</robot>
```

Spawn robot:

```bash
ros2 run gazebo_ros spawn_entity.py -entity bot -file robot.urdf
```

---

# ⭐ **2.8 Robot Navigation Around Table**

### Controller file:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move)
        self.step = 0

    def move(self):
        msg = Twist()

        # Circular motion
        msg.linear.x = 0.5
        msg.angular.z = 0.3

        self.pub.publish(msg)
        self.get_logger().info("Moving around table...")

def main():
    rclpy.init()
    nav = Navigator()
    rclpy.spin(nav)

if __name__ == '__main__':
    main()
```

Run:

```bash
ros2 run my_robot navigator
```

---

# ⭐ **2.9 Adding Sensors (LiDAR + Camera)**

Add to robot URDF:

### **Camera Sensor**

```xml
<gazebo reference="base_link">
  <sensor type="camera" name="my_camera">
    <update_rate>30</update_rate>
    <camera><horizontal_fov>1.0</horizontal_fov></camera>
  </sensor>
</gazebo>
```

### **LiDAR Sensor**

```xml
<gazebo reference="base_link">
  <sensor type="ray" name="laser">
    <ray>
      <scan><horizontal><samples>720</samples></horizontal></scan>
    </ray>
  </sensor>
</gazebo>
```

Now your robot can see and measure distances.

---

# ⭐ **2.10 Professional Gazebo Diagram**

```
                 +------------------------+
                 |      North Wall        |
                 +------------------------+
                 |                        |
                 |         Table          |
                 |          ███           |
                 |                        |
+----------------+                        +----------------+
| West Wall      |                        | East Wall      |
+----------------+                        +----------------+
                 |                        |
                 +------------------------+
                 |      South Wall        |
                 +------------------------+
```

---

# ⭐ **2.11 Full Practical Assignment (Pakistani Context)**

### 🎯 Task  
Build a simulation of a small room:

- Add Pakistani household-style walls  
- Add furniture (table + chair)  
- Add a mobile robot  
- Add a camera sensor  
- Add a LiDAR sensor  
- Make robot circle around the table  
- Save camera images  

### 📤 Submission Required:

- SDF world file  
- URDF robot file  
- Python control node  
- Screenshot of robot navigating  

---

# ⭐ **2.12 Quiz (Extended)**

<details>
<summary><strong>Q1: What is the default physics engine in Gazebo?</strong></summary>
A: ODE (Open Dynamics Engine)
</details>

<details>
<summary><strong>Q2: Which ROS topic controls robot velocity?</strong></summary>
A: `/cmd_vel`
</details>

<details>
<summary><strong>Q3: What is the difference between URDF and SDF?</strong></summary>
URDF = describes robot  
SDF = describes entire worlds (robots + environment)
</details>

<details>
<summary><strong>Q4: Which sensor is used for obstacle detection?</strong></summary>
A: LiDAR
</details>

---

# 🎉 **End of Expanded Module 2**
Ready for **Full Expanded Module 3: NVIDIA Isaac + AI Robotics**? 🚀  
Just say: **Generate Module 3 (long version)**  
