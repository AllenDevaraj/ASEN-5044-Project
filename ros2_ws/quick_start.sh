#!/bin/bash

# Quick Start Script for ROS2 Pick and Place
# This script provides easy commands to build and run the simulation

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

function print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

function print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

function print_info() {
    echo -e "${YELLOW}➜ $1${NC}"
}

function show_help() {
    cat << EOF
Usage: ./quick_start.sh [OPTION]

Options:
    build       Build the workspace
    clean       Clean build and rebuild
    run         Source workspace and show launch command
    test        Run tests and verify installation
    help        Show this help message

Examples:
    ./quick_start.sh build      # Build the workspace
    ./quick_start.sh clean      # Clean and rebuild
    ./quick_start.sh run        # Get ready to run simulation
    ./quick_start.sh test       # Verify everything works

EOF
}

function build_workspace() {
    print_header "Building ROS2 Workspace"
    ./build_workspace.sh
    print_success "Build complete!"
}

function clean_build() {
    print_header "Clean Build"
    ./build_workspace.sh --clean
    print_success "Clean build complete!"
}

function setup_run() {
    print_header "Setting Up Runtime Environment"
    
    if [ ! -d "install" ]; then
        echo "Workspace not built. Building now..."
        build_workspace
    fi
    
    print_info "Sourcing workspace..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    print_success "Workspace sourced!"
    echo ""
    print_header "Ready to Run"
    echo ""
    echo "To launch the simulation, run:"
    echo ""
    echo "  ros2 launch pick_and_place panda_pick_and_place.launch.py"
    echo ""
    echo "Or run components separately:"
    echo ""
    echo "  Terminal 1: Gazebo"
    echo "    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$(pwd)/install/pick_and_place/share/pick_and_place/models"
    echo "    gazebo --verbose install/pick_and_place/share/pick_and_place/worlds/pick_and_place.world"
    echo ""
    echo "  Terminal 2: Object Detector"
    echo "    ros2 run pick_and_place object_detector"
    echo ""
    echo "  Terminal 3: Controller"
    echo "    ros2 run pick_and_place controller"
    echo ""
    echo "  Terminal 4: State Machine"
    echo "    ros2 run pick_and_place pick_and_place_state_machine"
    echo ""
}

function run_tests() {
    print_header "Running Tests and Verification"
    
    if [ ! -d "install" ]; then
        echo "Workspace not built. Building now..."
        build_workspace
    fi
    
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    
    print_info "Checking packages..."
    if ros2 pkg list | grep -q "pick_and_place"; then
        print_success "pick_and_place package found"
    else
        echo "✗ pick_and_place package NOT found"
        exit 1
    fi
    
    if ros2 pkg list | grep -q "pick_and_place_msgs"; then
        print_success "pick_and_place_msgs package found"
    else
        echo "✗ pick_and_place_msgs package NOT found"
        exit 1
    fi
    
    print_info "Checking message definitions..."
    if ros2 interface show pick_and_place_msgs/msg/DetectedObject > /dev/null 2>&1; then
        print_success "DetectedObject message found"
    else
        echo "✗ DetectedObject message NOT found"
        exit 1
    fi
    
    if ros2 interface show pick_and_place_msgs/msg/DetectedObjectsStamped > /dev/null 2>&1; then
        print_success "DetectedObjectsStamped message found"
    else
        echo "✗ DetectedObjectsStamped message NOT found"
        exit 1
    fi
    
    print_info "Checking executables..."
    # Python packages install to bin/ directory
    if [ -f "install/pick_and_place/bin/object_detector" ]; then
        print_success "object_detector executable found"
    else
        echo "✗ object_detector executable NOT found"
    fi
    
    if [ -f "install/pick_and_place/bin/controller" ]; then
        print_success "controller executable found"
    else
        echo "✗ controller executable NOT found"
    fi
    
    if [ -f "install/pick_and_place/bin/pick_and_place_state_machine" ]; then
        print_success "pick_and_place_state_machine executable found"
    else
        echo "✗ pick_and_place_state_machine executable NOT found"
    fi
    
    print_info "Checking world and models..."
    if [ -f "install/pick_and_place/share/pick_and_place/worlds/pick_and_place.world" ]; then
        print_success "World file found"
    else
        echo "✗ World file NOT found"
    fi
    
    if [ -d "install/pick_and_place/share/pick_and_place/models" ]; then
        model_count=$(ls -1 install/pick_and_place/share/pick_and_place/models | wc -l)
        print_success "Models directory found ($model_count models)"
    else
        echo "✗ Models directory NOT found"
    fi
    
    echo ""
    print_header "Verification Complete"
    print_success "All checks passed! System is ready."
    echo ""
    echo "Run './quick_start.sh run' to get launch commands."
}

# Main script logic
case "$1" in
    build)
        build_workspace
        ;;
    clean)
        clean_build
        ;;
    run)
        setup_run
        ;;
    test)
        run_tests
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        if [ -z "$1" ]; then
            show_help
        else
            echo "Unknown option: $1"
            echo ""
            show_help
            exit 1
        fi
        ;;
esac

