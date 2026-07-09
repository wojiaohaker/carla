#!/bin/bash

# Auto-launch script for CARLA UE5 ROS2 Vehicle Control
# This script automatically sources the required environments before running

set -e

echo "=========================================="
echo "CARLA UE5 ROS2 Vehicle Control Launcher"
echo "=========================================="
echo ""

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "✗ Error: ROS2 Humble not found at /opt/ros/humble"
    echo "  Please install ROS2 Humble first."
    exit 1
fi

# Source carla_msgs
if [ -d "/home/qiyuan/Xiangrikui/carla_msgs/install" ]; then
    echo "✓ Sourcing carla_msgs..."
    source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash
else
    echo "✗ Error: carla_msgs not built"
    echo "  Building now..."
    cd /home/qiyuan/Xiangrikui/carla_msgs
    colcon build --packages-select carla_msgs
    source install/setup.bash
    echo "✓ carla_msgs built and sourced"
fi

# Verify imports
echo "✓ Verifying Python imports..."
python3 -c "from carla_msgs.msg import CarlaEgoVehicleControl" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "✗ Error: Cannot import carla_msgs"
    exit 1
fi

echo ""
echo "=========================================="
echo "Starting vehicle control script..."
echo "=========================================="
echo ""

# Run the actual script with all arguments
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/ros2_vehicle_control.py" "$@"
