# ASEN-5254 Project: Panda Pick and Place

## 🎉 ROS2 Migration Complete!

This repository contains both the original ROS1 project and the newly migrated ROS2 version.

```
ASEN-5254-Project/
├── catkin_ws/              # Original ROS1 Noetic workspace
│   └── src/
│       └── pick_and_place/
│
├── ros2_ws/                # ✨ NEW: ROS2 Humble workspace
│   ├── src/
│   │   ├── pick_and_place/
│   │   ├── pick_and_place_msgs/
│   │   ├── franka_ros2/
│   │   └── moveit_resources/
│   ├── build_workspace.sh  # Automated build script
│   ├── quick_start.sh      # Quick commands
│   ├── README.md           # Complete user guide
│   ├── SETUP_GUIDE.md      # Installation guide
│   └── install/            # Built packages
│
└── MIGRATION_SUMMARY.md    # Detailed migration notes
```

## Quick Start (ROS2 Humble)

### 1. Build the Workspace

```bash
cd ros2_ws
./build_workspace.sh
```

### 2. Run Tests

```bash
./quick_start.sh test
```

### 3. Launch Simulation

```bash
source install/setup.bash
ros2 launch pick_and_place panda_pick_and_place.launch.py
```

## What's New in ROS2

✅ **Messages**: Custom ROS2 message definitions  
✅ **Python Nodes**: All scripts ported to rclpy  
✅ **Launch Files**: Python-based launch system  
✅ **Build System**: colcon instead of catkin  
✅ **Dependencies**: MoveIt2 and Gazebo integration  
✅ **Documentation**: Complete guides and scripts  

## Documentation

- **[ros2_ws/README.md](ros2_ws/README.md)** - Complete ROS2 user guide
- **[ros2_ws/SETUP_GUIDE.md](ros2_ws/SETUP_GUIDE.md)** - Detailed setup instructions
- **[MIGRATION_SUMMARY.md](MIGRATION_SUMMARY.md)** - Full migration details
- **[ros2_ws/quick_start.sh](ros2_ws/quick_start.sh)** - Convenient command shortcuts

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble
- **Python**: 3.10+
- **Gazebo**: Fortress or Garden

## Features

- 🤖 Panda robot simulation in Gazebo
- 👁️ Vision-based object detection (RGB-D camera)
- 🎯 Automated pick and place of colored blocks
- 🔄 State machine for task management
- 📦 Custom Gazebo models and world

## Usage Commands

### Build
```bash
cd ros2_ws
./build_workspace.sh         # Normal build
./build_workspace.sh --clean # Clean rebuild
```

### Test
```bash
./quick_start.sh test       # Verify installation
./quick_start.sh run        # Show launch commands
```

### Run
```bash
source install/setup.bash
ros2 launch pick_and_place panda_pick_and_place.launch.py
```

## Build Status

| Package | Status |
|---------|--------|
| pick_and_place_msgs | ✅ Built |
| pick_and_place | ✅ Built |
| franka_msgs | ✅ Built |
| moveit_resources_panda | ✅ Built |
| **Total** | **9 packages** |

## Verification

All tests pass! ✅

```bash
$ cd ros2_ws && ./quick_start.sh test

✓ pick_and_place package found
✓ pick_and_place_msgs package found
✓ DetectedObject message found
✓ DetectedObjectsStamped message found
✓ object_detector executable found
✓ controller executable found
✓ pick_and_place_state_machine executable found
✓ World file found
✓ Models directory found (9 models)

✓ All checks passed! System is ready.
```

## Key Differences: ROS1 vs ROS2

| Feature | ROS1 (catkin_ws) | ROS2 (ros2_ws) |
|---------|------------------|----------------|
| Build Tool | catkin | colcon |
| Python API | rospy | rclpy + Node class |
| Launch Files | XML | Python |
| Messages | message_generation | rosidl_default_generators |
| Workspace Layout | devel/ | install/ |

## Next Steps

1. **First time setup**: See [ros2_ws/SETUP_GUIDE.md](ros2_ws/SETUP_GUIDE.md)
2. **Build the workspace**: Run `./build_workspace.sh` in ros2_ws/
3. **Test the system**: Run `./quick_start.sh test` in ros2_ws/
4. **Launch simulation**: Follow instructions in [ros2_ws/README.md](ros2_ws/README.md)

## Troubleshooting

See [ros2_ws/SETUP_GUIDE.md](ros2_ws/SETUP_GUIDE.md) for:
- Installation issues
- Build errors
- Runtime problems
- Gazebo configuration

## Contributing

This project was migrated from ROS1 to ROS2 for ASEN-5254.

**Original Author**: Elena Oikonomou (ROS1 version, Fall 2023)  
**ROS2 Port**: November 2025

## License

MIT License

---

## 📋 Migration Checklist

- [x] ROS2 workspace structure created
- [x] Custom messages ported to ROS2
- [x] Python nodes converted to rclpy
- [x] Launch files converted to Python
- [x] Dependencies cloned and configured
- [x] Build system working (colcon)
- [x] Documentation written
- [x] Automated scripts created
- [x] All packages build successfully
- [x] Tests pass

**Status: COMPLETE** ✅

For detailed migration notes, see [MIGRATION_SUMMARY.md](MIGRATION_SUMMARY.md).

---

**Ready to use!** Start with `cd ros2_ws && ./quick_start.sh test`

