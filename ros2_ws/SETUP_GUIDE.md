# ROS2 Pick and Place - Setup & Usage Guide

## System Requirements

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo (Fortress or Garden recommended for ROS2 Humble)
- Python 3.10+

## Installation

### 1. Install ROS2 Humble Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-visual-tools \
    ros-humble-geometric-shapes \
    ros-humble-control-toolbox \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    python3-pip \
    python3-colcon-common-extensions
```

### 2. Install Python Dependencies

```bash
pip3 install \
    opencv-python \
    numpy \
    python-statemachine
```

### 3. Install Additional Dependencies (Optional but Recommended)

```bash
# For franka_ros2
sudo apt install -y \
    ros-humble-generate-parameter-library \
    ros-humble-control-msgs \
    ros-humble-realtime-tools \
    libfranka-dev
```

### 4. Clone and Build Workspace

```bash
cd ~/ASEN-5254-Project/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Quick Start

### Build the Workspace

```bash
cd ~/ASEN-5254-Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch the Simulation

```bash
# Terminal 1: Launch Gazebo with the pick and place world
ros2 launch pick_and_place panda_pick_and_place.launch.py
```

If the above doesn't work immediately, you may need to launch components separately:

```bash
# Terminal 1: Start Gazebo with the world
gazebo --verbose ~/ASEN-5254-Project/ros2_ws/src/pick_and_place/worlds/pick_and_place.world

# Terminal 2: Spawn the robot (if needed)
ros2 run gazebo_ros spawn_entity.py -entity panda -topic robot_description

# Terminal 3: Start the object detector
ros2 run pick_and_place object_detector

# Terminal 4: Start the controller
ros2 run pick_and_place controller

# Terminal 5: Start the state machine
ros2 run pick_and_place pick_and_place_state_machine
```

## Troubleshooting

### Issue: Packages not found

**Solution**: Make sure you've sourced both ROS2 and your workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/ASEN-5254-Project/ros2_ws/install/setup.bash
```

### Issue: Build errors with franka_ros2

**Solution**: The franka_ros2 packages require hardware that might not be available. You can build specific packages:
```bash
# Build only the necessary packages
colcon build --packages-select pick_and_place_msgs pick_and_place franka_description
```

### Issue: Missing Python dependencies

**Solution**: Install missing dependencies:
```bash
pip3 install opencv-python numpy python-statemachine
```

### Issue: Gazebo models not loading

**Solution**: Set the Gazebo model path:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ASEN-5254-Project/ros2_ws/src/pick_and_place/models
```

Add this to your `~/.bashrc` to make it permanent:
```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ASEN-5254-Project/ros2_ws/src/pick_and_place/models' >> ~/.bashrc
```

### Issue: libfranka not found

**Solution**: You can skip franka_ros2 hardware packages if you're only doing simulation:
```bash
colcon build --packages-skip franka_hardware franka_bringup franka_robot_state_broadcaster
```

## Package Structure

```
ros2_ws/
├── src/
│   ├── pick_and_place_msgs/       # Custom message definitions
│   ├── pick_and_place/            # Main application package
│   │   ├── launch/                # Launch files
│   │   ├── models/                # Gazebo models
│   │   ├── worlds/                # Gazebo world files
│   │   └── pick_and_place/        # Python modules
│   ├── franka_ros2/               # Official Franka ROS2 packages
│   └── moveit_resources/          # MoveIt resources for Panda
├── build/                         # Build artifacts (auto-generated)
├── install/                       # Installed packages (auto-generated)
└── log/                          # Build logs (auto-generated)
```

## Next Steps

### Testing Individual Components

1. **Test message generation:**
   ```bash
   ros2 interface show pick_and_place_msgs/msg/DetectedObject
   ```

2. **Check available topics:**
   ```bash
   ros2 topic list
   ```

3. **Monitor object detection:**
   ```bash
   ros2 topic echo /object_detection
   ```

### Customization

- **Modify object positions**: Edit `src/pick_and_place/worlds/pick_and_place.world`
- **Adjust detection colors**: Edit `src/pick_and_place/pick_and_place/object_detector.py`
- **Change bin positions**: Edit `src/pick_and_place/pick_and_place/controller.py`

## Known Limitations

1. **MoveIt Integration**: The current version uses a simplified motion controller. Full MoveIt2 integration requires additional configuration.
   
2. **Hardware Interface**: This is a simulation-only setup. Running on real hardware requires the official franka_ros2 hardware interface.

3. **Camera Setup**: The camera is a static Kinect model. You may need to adjust camera parameters based on your specific setup.

## Getting Help

- Check build logs: `cat log/latest_build/events.log`
- View node output: Add `--ros-args --log-level debug` to any node command
- ROS2 Humble Documentation: https://docs.ros.org/en/humble/
- Franka ROS2: https://github.com/frankaemika/franka_ros2

## Converting from ROS1

This workspace was converted from the ROS1 catkin_ws. Key changes:

- **Build system**: `catkin` → `colcon`
- **Launch files**: XML → Python
- **Node initialization**: `rospy.init_node()` → `rclpy.init()` + `Node` class
- **Publishers/Subscribers**: ROS1 API → ROS2 API
- **Messages**: `message_generation` → `rosidl_default_generators`
- **TF**: `tf` → `tf2_ros`

