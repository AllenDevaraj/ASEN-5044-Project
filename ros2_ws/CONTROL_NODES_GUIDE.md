# 🎮 Control Nodes Guide

## ✅ **Your Existing Nodes:**

### **1. controller.py**
- **Purpose:** Motion control for pick-and-place
- **Needs:** MoveIt2 configured (not done yet)
- **Status:** ⚠️ Won't work fully without MoveIt2

### **2. object_detector.py**
- **Purpose:** Vision-based object detection
- **Needs:** Camera topics (already working!)
- **Status:** ✅ Should work now!

### **3. pick_and_place_state_machine.py**
- **Purpose:** Orchestrates pick-and-place workflow
- **Needs:** Controller + object detector working
- **Status:** ⚠️ Needs controller fixed first

---

## 🆕 **NEW: simple_peg_picker.py**

**What it does:**
- Controls panda1 to pick up orange peg
- Uses simple joint position control
- No MoveIt2 needed!
- Predefined joint angles

**Sequence:**
1. Move to neutral pose
2. Approach peg
3. Grasp peg position
4. Lift peg
5. Return to neutral

---

## 🚀 **How to Run - Step by Step:**

### **FIRST: Install Missing Controllers**

```bash
sudo apt install -y \
    ros-humble-joint-trajectory-controller \
    ros-humble-gripper-controllers
```

### **SECOND: Launch Gazebo with Robot**

```bash
# Terminal 1
cd ~/ASEN-5254-Project/ros2_ws
source install/setup.bash
ros2 launch panda_ign_description panda_ignition_demo.launch.py
```

**Wait for Gazebo to open and robot to appear!**

### **THIRD: Run the Simple Peg Picker**

```bash
# Terminal 2 (NEW terminal)
cd ~/ASEN-5254-Project/ros2_ws
source install/setup.bash
ros2 run pick_and_place simple_peg_picker
```

**Watch panda1 move and pick the peg!** 🎉

---

## 🔍 **What You'll See:**

**Terminal 2 Output:**
```
🤖 Simple Peg Picker Starting...
🎯 Starting peg picking sequence...
Step 1: Moving to neutral pose...
📤 Sending trajectory: [0.0, -0.785, 0.0]...
Step 2: Approaching peg...
📤 Sending trajectory: [0.5, -0.5, 0.3]...
Step 3: Moving to grasp peg...
📤 Sending trajectory: [0.6, -0.3, 0.4]...
Step 4: Lifting peg...
📤 Sending trajectory: [0.5, -0.6, 0.2]...
Step 5: Returning to neutral...
📤 Sending trajectory: [0.0, -0.785, 0.0]...
✅ Sequence complete!
```

**In Gazebo:**
- Panda1 arm moves through different poses
- Approaches the orange peg
- "Grasps" it (position-wise)
- Lifts and returns

---

## 📊 **Node Comparison:**

| Node | Works Now? | What It Does | How to Run |
|------|-----------|--------------|------------|
| `simple_peg_picker` | ✅ YES | Pick peg with panda1 | `ros2 run pick_and_place simple_peg_picker` |
| `object_detector` | ✅ YES | Detect colored blocks | `ros2 run pick_and_place object_detector` |
| `controller` | ⚠️ Partial | Full pick-place | `ros2 run pick_and_place controller` |
| `state_machine` | ❌ NO | Orchestrate tasks | Needs controller working first |

---

## 🎯 **Quick Test Commands:**

### **Test 1: Simple Peg Picker (NEW!)**
```bash
# Terminal 1: Launch Gazebo
ros2 launch panda_ign_description panda_ignition_demo.launch.py

# Terminal 2: Run peg picker
ros2 run pick_and_place simple_peg_picker
```

### **Test 2: Object Detector**
```bash
# Terminal 1: Gazebo (already running)

# Terminal 2: Run detector
ros2 run pick_and_place object_detector

# Terminal 3: Check detections
ros2 topic echo /object_detection
```

### **Test 3: Manual Joint Control**
```bash
# Move panda1's first joint
ros2 topic pub --once /panda_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
  points: [{positions: [0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5], time_from_start: {sec: 2}}]
}"
```

---

## 📝 **Where Are The Nodes?**

All nodes are in: `ros2_ws/src/pick_and_place/pick_and_place/`

**But you don't need to go there!** Just use `ros2 run`:

```bash
# Run from ANYWHERE after sourcing workspace
source install/setup.bash

ros2 run pick_and_place simple_peg_picker    # NEW simple controller
ros2 run pick_and_place object_detector      # Vision node
ros2 run pick_and_place controller           # Original controller
ros2 run pick_and_place pick_and_place_state_machine  # State machine
```

---

## 🔥 **TRY IT NOW!**

```bash
# First: Install controllers
sudo apt install -y ros-humble-joint-trajectory-controller ros-humble-gripper-controllers

# Then: Launch and test
# Terminal 1
ros2 launch panda_ign_description panda_ignition_demo.launch.py

# Terminal 2 (wait for Gazebo to fully load first!)
ros2 run pick_and_place simple_peg_picker
```

**Watch panda1 move and pick the orange peg!** 🤖🟧


