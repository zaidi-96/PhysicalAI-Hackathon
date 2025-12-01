# Module 3: NVIDIA Isaac Platform

## Chapter 3: AI-Powered Robotics

Robots are becoming smarter with AI. The NVIDIA Isaac platform allows engineers to **simulate, train, and test AI-powered robots** in photorealistic environments.

This module will cover:

1. NVIDIA Isaac introduction  
2. Installation via Docker  
3. Isaac Sim basics  
4. Robot simulation & sensors  
5. Python API examples  
6. AI model integration  
7. Practical exercises  
8. Quizzes  

---

# ⭐ **3.1 Introduction to NVIDIA Isaac**

### What is NVIDIA Isaac?

NVIDIA Isaac is a **platform for AI-driven robotics**, combining:

| Feature | Description |
|--------|-------------|
| Simulation | High-fidelity physics, photorealistic rendering |
| Perception | Camera, LiDAR, depth sensors |
| Navigation | Path planning, obstacle avoidance |
| AI Training | Reinforcement learning, imitation learning |
| ROS 2 Integration | Full interoperability with ROS 2 nodes |

---

### Why Isaac for Pakistani Students?

- Free trial for academics  
- Run on laptops with NVIDIA GPUs  
- Train robot models before buying hardware  
- Test warehouse robots, drones, and humanoids  

---

# ⭐ **3.2 Installation via Docker**

The easiest way to install Isaac Sim is using Docker.

### Step 1: Pull the Isaac Sim Docker Image

```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
```

> **Tip:** You must have NVIDIA Docker support installed.  

Check NVIDIA Docker:

```bash
docker run --rm --gpus all nvidia/cuda:12.1-base nvidia-smi
```

---

### Step 2: Run Isaac Sim Container

```bash
docker run --name isaac-sim \
  --gpus all \
  --privileged \
  -it -d \
  --network host \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Step 3: Access Container

```bash
docker exec -it isaac-sim bash
```

---

# ⭐ **3.3 Starting a Simple Simulation**

Inside the container:

```python
from omni.isaac.kit import SimulationApp

# Configuration
config = {"headless": False}  # True = no GUI
simulation_app = SimulationApp(config)

print("🤖 Isaac Sim is running!")
# Your Python robot code here

simulation_app.close()
```

---

# ⭐ **3.4 Features of Isaac Sim**

| Feature | Explanation |
|---------|-------------|
| Photorealistic Rendering | Train robots in realistic environments |
| Sensor Simulation | Camera, depth sensor, LiDAR |
| AI Training | Reinforcement Learning & Imitation Learning |
| ROS 2 Integration | Control robots via ROS 2 topics |
| Multi-Robot Support | Simulate multiple robots in one scene |

---

# ⭐ **3.5 Isaac Sim Python API**

## 3.5.1 Creating a Robot

```python
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.simulation_context import SimulationContext

sim = SimulationContext()
sim.initialize()

# Create a robot (box)
robot = DynamicCuboid(
    prim_path="/World/robot_box",
    name="robot_box",
    position=(0, 0, 0.5),
    size=(0.5, 0.3, 0.2),
)
```

## 3.5.2 Moving the Robot

```python
# Move robot forward
robot.set_world_pose((1.0, 0.0, 0.5))
```

## 3.5.3 Adding Sensors

```python
from omni.isaac.sensor import Camera

camera = Camera(prim_path="/World/robot_camera")
camera.initialize()
camera.enable()
```

---

# ⭐ **3.6 Integrating AI Models**

Isaac allows **reinforcement learning** and **perception-based tasks**:

### Example: Object Detection

```python
import torch
from torchvision.models import fasterrcnn_resnet50_fpn

# Load pre-trained model
model = fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()

# Input: image from Isaac Sim camera
# Output: bounding boxes of objects
```

### Example: Navigation

- Use camera + LiDAR data  
- Feed into AI model  
- Predict next motion (linear.x, angular.z)  

---

# ⭐ **3.7 Practical Exercises**

### Exercise 1: Simulate a Room

- Create a 10×10 meter room  
- Place obstacles (boxes, tables)  
- Spawn one robot  

### Exercise 2: Robot Navigation

- Move robot in a circle  
- Avoid obstacles using LiDAR  

### Exercise 3: Camera Integration

- Capture images from robot camera  
- Save images for training AI models  

### Exercise 4: ROS 2 Integration

- Publish robot velocity via `/cmd_vel`  
- Subscribe to `/camera/image_raw` for perception  

---

# ⭐ **3.8 Diagrams**

**Robot in Isaac Room**

```
      +-------------------------+
      |      North Wall         |
      +-------------------------+
      |                         |
      |   Table ███             |
      |       Robot ◉           |
      |                         |
+-----+-------------------------+-----+
| West Wall                       East Wall |
+------------------------------------------+
      |       South Wall         |
      +-------------------------+
```

---

# ⭐ **3.9 Quiz Section**

<details>
<summary>Q1: What are the main sensors supported by Isaac Sim?</summary>
**A:** Camera, LiDAR, depth sensors, IMU
</details>

<details>
<summary>Q2: How do you move a robot using Isaac Python API?</summary>
**A:** By setting its world pose or applying forces through DynamicCuboid / RigidBody objects
</details>

<details>
<summary>Q3: How to integrate ROS 2 with Isaac Sim?</summary>
**A:** Use ROS 2 topics, e.g., `/cmd_vel` to control robot and `/camera/image_raw` to get perception data
</details>

<details>
<summary>Q4: What is headless mode?</summary>
**A:** Running simulation without GUI, useful for AI training
</details>

---

# ⭐ **3.10 Next Steps**

After mastering Module 3, you can:

1. Train a robot using RL to avoid obstacles  
2. Integrate VLM (Module 4) for command-based robot actions  
3. Combine ROS 2, Isaac Sim, and AI for autonomous humanoid robot (Module 5)  

---

# 🎯 **End of Module 3**

> Module 3 is now **fully professional, step-by-step, and ready for Docusaurus integration**.  

