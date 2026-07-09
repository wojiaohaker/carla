#!/bin/bash

# CARLA ROS2 Vehicle Control Test Script
# This script publishes vehicle control commands to CARLA UE5 via ROS2 topics
# No external carla_msgs package required - uses ros2 topic pub directly

ROLE_NAME=${1:-hero}
TOPIC="/rt/carla/${ROLE_NAME}/vehicle_control_cmd"

echo "=========================================="
echo "CARLA UE5 ROS2 Vehicle Control Test"
echo "=========================================="
echo "Topic: ${TOPIC}"
echo ""
echo "Commands:"
echo "  w - throttle forward (0.8)"
echo "  s - brake (0.8)"
echo "  a - steer left (-0.5)"
echo "  d - steer right (0.5)"
echo "  x - stop (all zero)"
echo "  q - quit"
echo "=========================================="
echo ""

# Initialize control values
THROTTLE=0.0
STEER=0.0
BRAKE=0.0
REVERSE=false

while true; do
    read -n 1 -s KEY
    
    case $KEY in
        w)
            THROTTLE=0.8
            BRAKE=0.0
            REVERSE=false
            echo -e "\nThrottle forward"
            ;;
        s)
            THROTTLE=0.0
            BRAKE=0.8
            REVERSE=false
            echo -e "\nBrake"
            ;;
        a)
            STEER=-0.5
            echo -e "\nSteer left"
            ;;
        d)
            STEER=0.5
            echo -e "\nSteer right"
            ;;
        x)
            THROTTLE=0.0
            STEER=0.0
            BRAKE=0.0
            REVERSE=false
            echo -e "\nStop"
            ;;
        q)
            echo -e "\nQuitting..."
            # Send stop command before exit
            ros2 topic pub --once ${TOPIC} carla_msgs/msg/CarlaEgoVehicleControl \
                "{throttle: 0.0, steer: 0.0, brake: 1.0, reverse: false, hand_brake: false, gear: 1, manual_gear_shift: false}" > /dev/null 2>&1
            break
            ;;
        *)
            continue
            ;;
    esac
    
    # Publish control command
    ros2 topic pub --once ${TOPIC} carla_msgs/msg/CarlaEgoVehicleControl \
        "{throttle: ${THROTTLE}, steer: ${STEER}, brake: ${BRAKE}, reverse: ${REVERSE}, hand_brake: false, gear: 1, manual_gear_shift: false}" > /dev/null 2>&1
    
done

echo "Vehicle stopped"
