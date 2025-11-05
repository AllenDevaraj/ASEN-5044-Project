# ROS2 Pick and Place - Panda Robot

## Overview

This is a ROS2 Humble workspace for a pick-and-place application using the Franka Panda robot in Gazebo simulation. The project was converted from ROS1 (catkin_ws) to ROS2.

**Status**: ✅ Ready to build and test

## Features

- **Object Detection**: Vision-based detection of colored blocks (red, green, blue)
- **Pick and Place**: Automated pick-and-place using MoveIt2
- **State Machine**: Task planning and execution using python-statemachine
- **Gazebo Simulation**: Complete simulation environment with custom models
- **ROS2 Humble**: Fully ported to ROS2 with native message types and launch files

## Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble installed
- Python 3.10+

### Installation

1. **Install dependencies:**
```bash
sudo apt update && sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    python3-pip \
    python3-colcon-common-extensions

pip3 install opencv-python numpy python-statemachine
```

2. **Build the workspace:**
```bash
cd ~/ASEN-5254-Project/ros2_ws
./build_workspace.sh
```

3. **Source the workspace:**
```bash
source install/setup.bash
```

### Running the Simulation

**Option 1: Using the launch file (Recommended)**
```bash
source install/setup.bash
ros2 launch pick_and_place panda_pick_and_place.launch.py
```

**Option 2: Manual launch (for debugging)**

Terminal 1 - Gazebo:
```bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/pick_and_place/share/pick_and_place/models
gazebo --verbose $(pwd)/install/pick_and_place/share/pick_and_place/worlds/pick_and_place.world
```

Terminal 2 - Object Detector:
```bash
source install/setup.bash
ros2 run pick_and_place object_detector
```

Terminal 3 - Controller:
```bash
source install/setup.bash
ros2 run pick_and_place controller
```

Terminal 4 - State Machine:
```bash
source install/setup.bash
ros2 run pick_and_place pick_and_place_state_machine
```

## Workspace Structure

```
ros2_ws/
├── src/
│   ├── pick_and_place_msgs/          # Custom ROS2 messages
│   │   ├── msg/
│   │   │   ├── DetectedObject.msg
│   │   │   └── DetectedObjectsStamped.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── pick_and_place/               # Main application
│   │   ├── pick_and_place/           # Python package
│   │   │   ├── __init__.py
│   │   │   ├── object_detector.py    # Vision-based object detection
│   │   │   ├── controller.py         # Robot motion controller
│   │   │   └── pick_and_place_state_machine.py  # Task state machine
│   │   ├── launch/
│   │   │   └── panda_pick_and_place.launch.py
│   │   ├── worlds/
│   │   │   └── pick_and_place.world
│   │   ├── models/                   # Gazebo models
│   │   │   ├── bin_red/
│   │   │   ├── bin_green/
│   │   │   ├── bin_blue/
│   │   │   ├── block_red/
│   │   │   ├── block_green/
│   │   │   ├── block_blue/
│   │   │   ├── workbench/
│   │   │   ├── underbin_bench/
│   │   │   └── kinect/
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── franka_ros2/                  # Official Franka ROS2 packages
│   │   └── franka_msgs/              # (only messages built)
│   │
│   └── moveit_resources/             # MoveIt2 resources
│       ├── panda_description/        # Panda URDF/meshes
│       └── panda_moveit_config/      # MoveIt config for Panda
│
├── build/                            # Build artifacts (auto-generated)
├── install/                          # Installed packages (auto-generated)
├── log/                              # Build logs (auto-generated)
├── build_workspace.sh                # Build script
├── README.md                         # This file
└── SETUP_GUIDE.md                    # Detailed setup instructions
```

## Package Descriptions

### pick_and_place_msgs
Custom ROS2 message definitions for detected objects:
- `DetectedObject.msg`: Position, dimensions, and color of a detected object
- `DetectedObjectsStamped.msg`: Array of detected objects with timestamp

### pick_and_place
Main application package containing:
- **object_detector.py**: Uses OpenCV and depth sensing to detect colored blocks on the workbench
- **controller.py**: Controls the Panda robot arm using MoveIt2 for pick-and-place operations
- **pick_and_place_state_machine.py**: Orchestrates the pick-and-place workflow using a state machine

## ROS2 Topics

### Published Topics
- `/object_detection` (`pick_and_place_msgs/DetectedObjectsStamped`): Detected objects with their 3D positions

### Subscribed Topics
- `/camera/color/image_raw` (`sensor_msgs/Image`): RGB camera feed
- `/camera/depth/image_raw` (`sensor_msgs/Image`): Depth camera feed
- `/camera/color/camera_info` (`sensor_msgs/CameraInfo`): Camera parameters
- `/joint_states` (`sensor_msgs/JointState`): Robot joint states

### Services Used
- `/gazebo/get_entity_state` (Gazebo): Get model positions from simulation

## Configuration

### Object Detection Colors
Edit `src/pick_and_place/pick_and_place/object_detector.py`:
```python
self.color_ranges = {
    "blue": [np.array([110,50,50]), np.array([130,255,255])],
    "green": [np.array([36, 25, 25]), np.array([70, 255,255])],
    "red": [np.array([0, 100, 100]), np.array([10, 255, 255])]
}
```

### Bin Positions
Edit `src/pick_and_place/pick_and_place/controller.py`:
```python
self.red_bin = (-0.5, -0.25)
self.green_bin = (-0.5, 0.0)
self.blue_bin = (-0.5, 0.25)
```

### World Configuration
Edit `src/pick_and_place/worlds/pick_and_place.world` to modify:
- Object initial positions
- Camera placement
- Lighting conditions

## Troubleshooting

### Build Errors

**Problem**: `libfranka not found`
```
Solution: Hardware packages are skipped automatically. This is expected for simulation-only setup.
```

**Problem**: Package not found after build
```bash
# Solution: Source the workspace
source install/setup.bash
```

**Problem**: Build fails with dependency errors
```bash
# Solution: Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Runtime Errors

**Problem**: Gazebo models not loading
```bash
# Solution: Set the model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/install/pick_and_place/share/pick_and_place/models
```

**Problem**: Camera image not publishing
```
Solution: Check that Gazebo is running and the camera sensor is active in the world file.
```

**Problem**: Robot not moving
```
Solution: Ensure ros2_control and the robot controllers are properly configured and loaded.
```

### Clean Build

If you encounter persistent build issues:
```bash
./build_workspace.sh --clean
```

## Development

### Adding New Objects

1. Create a new model in `src/pick_and_place/models/`
2. Add the model to the world file in `src/pick_and_place/worlds/pick_and_place.world`
3. Add color detection range in `object_detector.py` if needed

### Modifying Pick-and-Place Logic

1. **State Machine**: Edit `pick_and_place_state_machine.py`
2. **Motion Planning**: Edit `controller.py`
3. **Vision Processing**: Edit `object_detector.py`

### Rebuild After Changes

```bash
colcon build --packages-select pick_and_place --symlink-install
source install/setup.bash
```

## Testing

### Test Message Generation
```bash
ros2 interface show pick_and_place_msgs/msg/DetectedObject
```

### Monitor Topics
```bash
# List all topics
ros2 topic list

# View detected objects
ros2 topic echo /object_detection

# View camera images (requires image_view or rqt)
ros2 run rqt_image_view rqt_image_view
```

### Check Node Status
```bash
ros2 node list
ros2 node info /vision_object_detector
```

## Known Limitations

1. **MoveIt Integration**: Simplified motion control. Full MoveIt2 planning requires additional configuration for the Panda robot with ros2_control.

2. **Gripper Control**: Uses basic gripper commands. Advanced grasping requires full franka_gripper integration.

3. **Hardware Support**: This is a simulation-only setup. Real hardware requires franka_hardware packages and proper network configuration.

4. **Camera Calibration**: Using default parameters. Fine-tuning camera intrinsics may improve detection accuracy.

## Migration Notes (ROS1 → ROS2)

### Key Changes

| Aspect | ROS1 | ROS2 |
|--------|------|------|
| Build Tool | catkin | colcon |
| Python API | rospy | rclpy |
| Launch Files | XML | Python |
| Message Gen | message_generation | rosidl_default_generators |
| TF | tf | tf2_ros |
| Parameters | param server | node parameters |
| Node Init | rospy.init_node() | Node class + rclpy.init() |

### Code Changes

- `rospy.Subscriber()` → `self.create_subscription()`
- `rospy.Publisher()` → `self.create_publisher()`
- `rospy.Service()` → `self.create_service()`
- `rospy.Rate()` → `self.create_rate()`
- `rospy.sleep()` → `self.get_clock().sleep_for()`
- `rospy.Time.now()` → `self.get_clock().now()`
- `rospy.loginfo()` → `self.get_logger().info()`

## Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Franka ROS2 GitHub](https://github.com/frankaemika/franka_ros2)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/)
- [Gazebo Documentation](https://gazebosim.org/)
- [Original ROS1 Project](../catkin_ws/)

## Authors

- **Elena Oikonomou** - Original ROS1 implementation (Fall 2023)
- **ROS2 Port** - ASEN-5254 Project (2025)

## License

MIT License
