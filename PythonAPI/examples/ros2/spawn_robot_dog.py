#!/usr/bin/env python3

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
# de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Spawns the robot dog (blueprint.robot_dog -> BP_ThirdPersonCharacter) together
with a ray-cast lidar matching the original Gazebo gpu_lidar, for the
Gazebo -> CarlaUE5 navigation migration.

ROS 2 side effects (handled by the simulator, not by this script):
  * subscribes  /cmd_vel                  (geometry_msgs/msg/Twist)
  * publishes   /odom                     (nav_msgs/msg/Odometry, 50 Hz)
  * publishes   /tf                       odom->base_link + base_link->laser_up
  * publishes   /scan/points              (sensor_msgs/msg/PointCloud2, frame laser_up)

Usage:
    python3 spawn_robot_dog.py [--host HOST] [--port PORT]
                               [-x X] [-y Y] [-z Z] [--yaw YAW_DEG]
                               [--timeout SECONDS]
"""

import argparse
import math
import sys

try:
    sys.path.append('..')
    import carla
except ImportError:
    raise RuntimeError('cannot import "carla" (see PythonAPI/carla)')


# Lidar spec mirrored from agibot_d1_top_description/urdf/top.urdf:
#   640 horizontal samples over 360 deg, 32 channels,
#   vertical FOV [-0.261799, 0.4] rad, range [0.08, 20] m, 10 Hz.
LIDAR_CHANNELS = 32
LIDAR_RANGE = 20.0
LIDAR_ROTATION_FREQUENCY = 10.0
LIDAR_POINTS_PER_SECOND = 6400  # 640 samples * 10 Hz
LIDAR_UPPER_FOV = math.degrees(0.4)        # ~22.92 deg
LIDAR_LOWER_FOV = math.degrees(-0.261799)  # ~-15 deg
LIDAR_SENSOR_TICK = 0.1                    # 10 Hz

# laser_up_joint offset relative to base_link (0, 0, 0.15) m.
LIDAR_OFFSET_Z_CM = 15.0


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host', metavar='H', default='127.0.0.1',
                           help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-x', default=0.0, type=float,
                           help='spawn X location [m] (default: 0.0)')
    argparser.add_argument('-y', default=-14.9, type=float,
                           help='spawn Y location [m] (default: -14.9)')
    argparser.add_argument('-z', default=0.4, type=float,
                           help='spawn Z location [m] (default: 0.4)')
    argparser.add_argument('--yaw', default=0.0, type=float,
                           help='spawn yaw [deg, ROS CCW positive] (default: 0.0)')
    argparser.add_argument('--timeout', metavar='T', default=10.0, type=float,
                           help='client connection timeout in seconds')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # --- Robot dog -----------------------------------------------------------
    dog_bp = blueprint_library.find('blueprint.robot_dog')
    # ROS convention: CCW yaw in radians -> UE clockwise yaw in degrees.
    transform = carla.Transform(
        carla.Location(x=args.x, y=-args.y, z=args.z),
        carla.Rotation(yaw=-args.yaw))
    dog = world.spawn_actor(dog_bp, transform)
    if dog is None:
        raise RuntimeError('failed to spawn blueprint.robot_dog')
    print('spawned robot dog:', dog.id, 'at', transform.location)

    # --- Lidar on the laser_up frame -----------------------------------------
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('channels', str(LIDAR_CHANNELS))
    lidar_bp.set_attribute('range', str(LIDAR_RANGE))
    lidar_bp.set_attribute('rotation_frequency', str(LIDAR_ROTATION_FREQUENCY))
    lidar_bp.set_attribute('points_per_second', str(LIDAR_POINTS_PER_SECOND))
    lidar_bp.set_attribute('upper_fov', str(LIDAR_UPPER_FOV))
    lidar_bp.set_attribute('lower_fov', str(LIDAR_LOWER_FOV))
    lidar_bp.set_attribute('sensor_tick', str(LIDAR_SENSOR_TICK))
    # Deterministic output for SLAM: no dropoff / noise.
    lidar_bp.set_attribute('dropoff_general_rate', '0.0')
    lidar_bp.set_attribute('noise_stddev', '0.0')

    lidar_transform = carla.Transform(carla.Location(z=LIDAR_OFFSET_Z_CM / 100.0))
    lidar = world.spawn_actor(
        lidar_bp, lidar_transform, attach_to=dog,
        attachment_type=carla.AttachmentType.Rigid)
    if lidar is None:
        dog.destroy()
        raise RuntimeError('failed to spawn lidar on the robot dog')
    print('attached lidar (frame laser_up) at +%.2f m' % (LIDAR_OFFSET_Z_CM / 100.0))

    print('\nROS 2 topics provided by the simulator:')
    print('  sub: /cmd_vel (geometry_msgs/msg/Twist)')
    print('  pub: /odom, /tf, /scan/points (frame laser_up)')
    print('\nCtrl+C to destroy the actors and exit.')

    try:
        while True:
            world.wait_for_tick()
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        lidar.destroy()
        dog.destroy()
        print('actors destroyed')


if __name__ == '__main__':
    main()
