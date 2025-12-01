# Module 4: Vision-Language Models (VLMs) for Robotics

## Chapter 4: AI for Robot Perception

Vision-Language Models (VLMs) combine **computer vision** and **natural language processing** to enable robots to understand their environment and follow instructions given in natural language. This module will guide students through:

1. Introduction to VLMs  
2. Architecture and workflow  
3. Simple Python implementation  
4. Training strategies  
5. Integration with robot control  
6. Practical exercises  
7. Diagrams for understanding  
8. Quizzes  

---

# ⭐ 4.1 What are Vision-Language Models (VLMs)?

VLMs are AI models that understand **images** and **text simultaneously**. They allow robots to:

- Detect objects from text commands  
- Navigate based on spoken or written instructions  
- Perform complex manipulation tasks  

**Examples in robotics:**

| Task | VLM Use Case |
|------|--------------|
| Object Detection | "Pick up the red cube" |
| Navigation | "Go to the nearest chair" |
| Manipulation | "Place the box on the table" |

---

# ⭐ 4.2 Architecture of VLMs

A typical VLM consists of **three main components**:

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Vision    │    │  Language   │    │   Action    │
│  Encoder    │    │  Encoder    │    │  Decoder    │
└──────┬──────┘    └──────┬──────┘    └──────┬──────┘
       │                  │                   │
    Camera ──────────▶ "Pick up" ───────▶ Robot Arm
      Input              Command              Motion
```

**Description:**

- **Vision Encoder:** Processes images from cameras, LiDAR, or depth sensors  
- **Language Encoder:** Converts text commands into vector embeddings  
- **Action Decoder:** Converts combined embeddings into robot actions (linear/angular velocity, manipulator movement)

---

# ⭐ 4.3 Simple Python Implementation

This example uses **PyTorch** for a small VLM:

```python
import torch
import torch.nn as nn

class SimpleVLM(nn.Module):
    def __init__(self, vision_dim=512, language_dim=256, action_dim=4):
        super().__init__()
        # Vision model
        self.vision_encoder = nn.Sequential(
            nn.Conv2d(3, 16, 3, stride=2),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(16*31*31, vision_dim)
        )
        # Language model
        self.language_encoder = nn.Embedding(1000, language_dim)
        # Fusion layer
        self.fc = nn.Linear(vision_dim + language_dim, 128)
        self.action_decoder = nn.Linear(128, action_dim)
    
    def forward(self, image, command):
        vision_feat = self.vision_encoder(image)
        command_feat = self.language_encoder(command).mean(dim=1)
        combined = torch.cat([vision_feat, command_feat], dim=1)
        x = torch.relu(self.fc(combined))
        action = self.action_decoder(x)
        return action
```

**Explanation:**

- `image`: Robot camera input  
- `command`: Tokenized text command  
- `action`: Output vector `[linear.x, linear.y, angular.z, gripper]`  

---

# ⭐ 4.4 Training VLMs for Robotics

**Step 1: Collect Dataset**

- Images or videos from robot cameras  
- Commands in natural language  
- Corresponding robot actions  

**Step 2: Preprocessing**

- Resize and normalize images  
- Tokenize text commands  
- Normalize action vectors  

**Step 3: Training Loop**

```python
optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
criterion = nn.MSELoss()  # For continuous actions

for epoch in range(100):
    for img, cmd, act in dataloader:
        optimizer.zero_grad()
        pred = model(img, cmd)
        loss = criterion(pred, act)
        loss.backward()
        optimizer.step()
    print(f"Epoch {epoch} Loss: {loss.item():.4f}")
```

**Step 4: Evaluation**

- Test robot in simulation environment (e.g., Isaac Sim)  
- Measure task success: object picked, navigation accuracy, etc.

---

# ⭐ 4.5 Integration with Robot Control

After training, the model can **control a robot in real-time**:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VLMRobotController(Node):
    def __init__(self, vlm_model):
        super().__init__('vlm_robot')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.model = vlm_model
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        # Capture camera image and tokenized command
        image = get_camera_frame()
        command = tokenize("Move to the red box")
        
        # Predict action
        action = self.model(image, command)
        
        msg = Twist()
        msg.linear.x = action[0].item()
        msg.linear.y = action[1].item()
        msg.angular.z = action[2].item()
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f"Moving robot: {msg}")

def main():
    rclpy.init()
    model = SimpleVLM()
    controller = VLMRobotController(model)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

---

# ⭐ 4.6 Practical Exercises

### Exercise 1: Object Detection from Commands
- Place multiple objects in simulation  
- Give commands like `"Pick up the green cube"`  
- Robot should identify and move toward the object

### Exercise 2: Navigation with Voice/Text
- Map a room  
- Use VLM to convert `"Go to the table"` into motion commands  
- Avoid obstacles

### Exercise 3: Pick-and-Place
- Use VLM to manipulate objects: grasp, lift, and place  
- Combine perception and action decoder  

### Exercise 4: Multi-Robot Coordination
- Two robots share camera feeds  
- Execute commands collaboratively  

---

# ⭐ 4.7 Diagrams

**VLM Control Loop**

```
[Camera Image] ─┐
                │
[Text Command] ─┘
        │
   ┌──────────────┐
   │   VLM Model  │
   └─────┬────────┘
         │
[Predicted Actions]
         │
     ┌─────────┐
     │ Robot   │
     │ Control │
     └─────────┘
```

**Robot Task Example: Pick-and-Place**

```
[Command]: "Pick up red cube"
      │
      ▼
[Vision] → [Locate Cube] → [Calculate Path] → [Execute Motion]
      │
      ▼
[Robot Arm Grasps Cube] → [Robot Moves] → [Cube Placed on Table]
```

---

# ⭐ 4.8 Quiz Section

<details>
<summary>Q1: What is the role of the Vision Encoder in a VLM?</summary>
**A:** Processes images from robot cameras to extract features.
</details>

<details>
<summary>Q2: How does the Language Encoder work?</summary>
**A:** Converts text commands into embeddings the robot can understand.
</details>

<details>
<summary>Q3: What are action decoders used for?</summary>
**A:** Converts the combined visual + language embeddings into robot motions.
</details>

<details>
<summary>Q4: Name two practical applications of VLMs in robotics.</summary>
**A:** Object manipulation from commands, navigation using natural language.
</details>

<details>
<summary>Q5: How do you integrate a trained VLM with ROS 2?</summary>
**A:** Use the model output to publish velocity or manipulator commands via ROS 2 topics like `/cmd_vel` or `/arm_controller`.
</details>

---

# ⭐ 4.9 Next Steps

After completing Module 4:

1. Integrate your VLM with **Isaac Sim** (Module 3)  
2. Combine with **ROS 2 nodes** (Module 1)  
3. Apply in **autonomous humanoid robots** (Module 5)  
4. Train more complex VLMs with **reinforcement learning** for adaptive behaviors  

---

# 🎯 **End of Module 4**

> Module 4 now contains **full theory, practical examples, Python implementation, diagrams, exercises, and quizzes**, ready for your Docusaurus textbook.
