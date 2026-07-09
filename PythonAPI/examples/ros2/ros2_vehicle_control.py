#!/usr/bin/env python3

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS2 Vehicle Control Demo

This script publishes vehicle control commands to CARLA via ROS2 topics.
It demonstrates how to control a CARLA vehicle using ROS2 messages instead of
the traditional Python API.

IMPORTANT: Before running this script, make sure to source the ROS2 environment:
    source /opt/ros/humble/setup.bash
    source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash

Usage:
    python3 ros2_vehicle_control.py --role-name hero
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from carla_msgs.msg import CarlaEgoVehicleControl
except ImportError:
    print("Error: ROS2 and carla_msgs package required.")
    print("\nSetup instructions:")
    print("  1. Install colcon:")
    print("     sudo apt install python3-colcon-common-extensions")
    print("\n  2. Build carla_msgs:")
    print("     cd /home/qiyuan/Xiangrikui/carla_msgs")
    print("     colcon build --packages-select carla_msgs")
    print("\n  3. Source the environment:")
    print("     source /opt/ros/humble/setup.bash")
    print("     source /home/qiyuan/Xiangrikui/carla_msgs/install/setup.bash")
    sys.exit(1)


class CarlaVehicleController(Node):
    """ROS2 node that publishes vehicle control commands to CARLA."""

    def __init__(self, role_name='hero'):
        super().__init__('carla_vehicle_controller')
        
        self.role_name = role_name
        
        # Publisher for vehicle control commands
        # Topic name matches CARLA UE5's CarlaEgoVehicleControlSubscriber
        # Format: /carla/{role_name}/vehicle_control_cmd
        topic_name = f'/carla/{role_name}/vehicle_control_cmd'
        
        self.publisher_ = self.create_publisher(
            CarlaEgoVehicleControl,
            topic_name,
            10  # QoS history depth
        )
        
        self.get_logger().info(f'CARLA UE5 ROS2 Vehicle Control')
        self.get_logger().info(f'Publishing to: {topic_name}')
        self.get_logger().info('')
        self.get_logger().info('Keyboard Controls:')
        self.get_logger().info('  w - Throttle forward (0.8)')
        self.get_logger().info('  s - Brake (0.8)')
        self.get_logger().info('  a - Steer left (-0.5)')
        self.get_logger().info('  d - Steer right (0.5)')
        self.get_logger().info('  x - Stop (all zero)')
        self.get_logger().info('  q - Quit')
        self.get_logger().info('')
        self.get_logger().info('Type a command and press Enter...')
        
        # Current control state
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.reverse = False
        
        # Start keyboard input loop in a separate thread
        import threading
        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def publish_control(self):
        """Publish current control state to CARLA."""
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle = self.throttle
        msg.steer = self.steer
        msg.brake = self.brake
        msg.reverse = self.reverse
        msg.hand_brake = False
        msg.gear = 1 if not self.reverse else -1
        msg.manual_gear_shift = False
        
        self.publisher_.publish(msg)
        self.get_logger().debug(
            f'Control: throttle={self.throttle:.2f}, steer={self.steer:.2f}, '
            f'brake={self.brake:.2f}, reverse={self.reverse}'
        )

    def keyboard_loop(self):
        """Read keyboard input and update control state."""
        try:
            while True:
                key = input().lower().strip()
                
                if key == 'q':
                    self.get_logger().info('Quitting...')
                    # Send stop command before exiting
                    self.throttle = 0.0
                    self.steer = 0.0
                    self.brake = 1.0
                    self.publish_control()
                    rclpy.shutdown()
                    break
                
                elif key == 'w':
                    # Throttle forward
                    self.throttle = 0.8
                    self.brake = 0.0
                    self.reverse = False
                    self.get_logger().info('Throttle forward')
                    
                elif key == 's':
                    # Brake (not reverse - use gear for reverse)
                    self.throttle = 0.0
                    self.brake = 0.8
                    self.reverse = False
                    self.get_logger().info('Brake')
                    
                elif key == 'a':
                    # Steer left
                    self.steer = -0.5
                    self.get_logger().info('Steer left')
                    
                elif key == 'd':
                    # Steer right
                    self.steer = 0.5
                    self.get_logger().info('Steer right')
                    
                elif key == 'x':
                    # Stop
                    self.throttle = 0.0
                    self.steer = 0.0
                    self.brake = 0.0
                    self.reverse = False
                    self.get_logger().info('Stop')
                
                # Publish the control command
                self.publish_control()
                
        except EOFError:
            pass
        except Exception as e:
            self.get_logger().error(f'Input error: {e}')

    def run(self):
        """Main loop - keep publishing at regular intervals."""
        rate = self.create_rate(20)  # 20 Hz
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                # Continuously publish last command to maintain control
                # (CARLA expects continuous commands)
                self.publish_control()
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            # Send stop command on exit
            self.throttle = 0.0
            self.steer = 0.0
            self.brake = 1.0
            self.publish_control()
            self.get_logger().info('Vehicle stopped')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='CARLA UE5 ROS2 Vehicle Control Demo\n'
                    'Controls a CARLA vehicle via ROS2 topics without carla_bridge.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 ros2_vehicle_control.py --role-name hero
  python3 ros2_vehicle_control.py --role-name ego_vehicle

Note: Make sure CARLA UE5 is running and has registered a vehicle with the specified role_name.
        """
    )
    parser.add_argument('--role-name', default='hero',
                       help='Vehicle role name in CARLA (default: hero)')
    
    args = parser.parse_args(args)
    
    print('\n' + '='*60)
    print('CARLA UE5 ROS2 Vehicle Control Demo')
    print('='*60)
    print(f'Role name: {args.role_name}')
    print(f'Topic: /carla/{args.role_name}/vehicle_control_cmd')
    print('='*60 + '\n')
    
    # Initialize ROS2 (pass None or a list, not argparse Namespace)
    rclpy.init()
    
    try:
        controller = CarlaVehicleController(role_name=args.role_name)
        controller.run()
    except KeyboardInterrupt:
        print('\nCancelled by user.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
