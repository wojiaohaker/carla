#!/bin/bash

# Quick test script to verify CARLA UE5 ROS2 vehicle control setup

echo "=========================================="
echo "CARLA UE5 ROS2 Control - Setup Verification"
echo "=========================================="
echo ""

# Check 1: ROS2 environment
echo "✓ Checking ROS2 environment..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✓ ROS2 Humble found"
else
    echo "  ✗ ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Check 2: carla_msgs
echo ""
echo "✓ Checking carla_msgs..."
if [ -d "/home/qiyuan/Xiangrikui/carla_msgs/install" ]; then
    source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash
    echo "  ✓ carla_msgs found"
    
    # Verify message type
    if ros2 interface show carla_msgs/msg/CarlaEgoVehicleControl > /dev/null 2>&1; then
        echo "  ✓ CarlaEgoVehicleControl message type available"
    else
        echo "  ✗ CarlaEgoVehicleControl message type not found"
        echo "  → Try rebuilding: cd /home/qiyuan/Xiangrikui/carla_msgs && colcon build"
        exit 1
    fi
else
    echo "  ✗ carla_msgs not built"
    echo "  → Building now..."
    cd /home/qiyuan/Xiangrikui/carla_msgs
    colcon build --packages-select carla_msgs
    if [ $? -eq 0 ]; then
        source install/setup.bash
        echo "  ✓ carla_msgs built successfully"
    else
        echo "  ✗ Failed to build carla_msgs"
        exit 1
    fi
fi

# Check 3: Python import
echo ""
echo "✓ Checking Python imports..."
python3 -c "from carla_msgs.msg import CarlaEgoVehicleControl" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "  ✓ Python can import carla_msgs"
else
    echo "  ✗ Python cannot import carla_msgs"
    echo "  → Make sure you sourced the environment:"
    echo "     source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash"
    exit 1
fi

# Check 4: Script exists
echo ""
echo "✓ Checking control script..."
SCRIPT_PATH="/home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2/ros2_vehicle_control.py"
if [ -f "$SCRIPT_PATH" ]; then
    echo "  ✓ ros2_vehicle_control.py found"
else
    echo "  ✗ ros2_vehicle_control.py not found"
    exit 1
fi

# Summary
echo ""
echo "=========================================="
echo "✓ All checks passed!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Start CARLA UE5:"
echo "     cd /home/qiyuan/UnrealEngine/CarlaUE5/Build"
echo "     cmake --build Build --target launch"
echo ""
echo "  2. Run the control script:"
echo "     python3 $SCRIPT_PATH --role-name hero"
echo ""
echo "  3. Or test with ros2 topic pub:"
echo "     ros2 topic pub --once /rt/carla/hero/vehicle_control_cmd \\"
echo "         carla_msgs/msg/CarlaEgoVehicleControl \\"
echo "         \"{throttle: 0.5, steer: 0.0, brake: 0.0, reverse: false}\""
echo ""
echo "For detailed documentation, see:"
echo "  /home/qiyuan/UnrealEngine/CarlaUE5/PythonAPI/examples/ros2/README_VEHICLE_CONTROL.md"
echo ""
