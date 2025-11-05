#!/bin/bash

# Build script for ROS2 Pick and Place Workspace
# This script builds only the necessary packages for the simulation

set -e

echo "========================================"
echo "Building ROS2 Pick and Place Workspace"
echo "========================================"

# Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble sourced"
else
    echo "✗ ROS2 Humble not found. Please install ROS2 Humble first."
    exit 1
fi

# Navigate to workspace
cd "$(dirname "$0")"
echo "✓ In workspace: $(pwd)"

# Clean previous builds (optional)
if [ "$1" == "--clean" ]; then
    echo "Cleaning previous builds..."
    rm -rf build/ install/ log/
fi

# List of packages to skip (hardware-specific and their dependencies)
SKIP_PACKAGES="franka_hardware franka_bringup franka_robot_state_broadcaster integration_launch_testing franka_ign_ros2_control franka_semantic_components franka_gripper franka_example_controllers franka_gazebo_bringup franka_ros2 franka_fr3_moveit_config"

# Build the workspace
echo ""
echo "Building workspace..."
echo "Skipping hardware packages: $SKIP_PACKAGES"
echo ""

colcon build \
    --packages-skip $SKIP_PACKAGES \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================"
    echo "✓ Build successful!"
    echo "========================================"
    echo ""
    echo "To use the workspace, run:"
    echo "  source install/setup.bash"
    echo ""
    echo "To launch the simulation, run:"
    echo "  ros2 launch pick_and_place panda_pick_and_place.launch.py"
else
    echo ""
    echo "========================================"
    echo "✗ Build failed"
    echo "========================================"
    exit 1
fi

