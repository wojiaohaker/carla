#!/usr/bin/env python

"""Minimal movement diagnostic for the EastLUV3151 vehicle.

Spawns vehicle.eastluv3151.luv3151, applies full throttle in a straight
line for a few seconds and reports speed / position. No agent, no HUD,
no autopilot -- it isolates the client->server control path from the
vehicle physics configuration.

Usage:
    python3 test_eastluv_move.py [--sync]
"""

import argparse
import sys
import time

import carla

TARGET_VEHICLE_BLUEPRINT = 'vehicle.eastluv3151.luv3151'


def main():
    parser = argparse.ArgumentParser(description='EastLUV3151 movement diagnostic')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', default=2000, type=int)
    parser.add_argument('--sync', action='store_true',
                        help='Run the world in synchronous mode')
    parser.add_argument('--seconds', default=5.0, type=float,
                        help='Throttle duration in seconds (default: 5)')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()

    original_settings = world.get_settings()
    if args.sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

    vehicle = None
    try:
        blueprints = world.get_blueprint_library().filter(TARGET_VEHICLE_BLUEPRINT)
        if not blueprints:
            print("ERROR: blueprint '%s' not found." % TARGET_VEHICLE_BLUEPRINT)
            return 1
        blueprint = blueprints[0]
        blueprint.set_attribute('role_name', 'diag')
        if blueprint.has_attribute('color'):
            blueprint.set_attribute(
                'color', blueprint.get_attribute('color').recommended_values[0])

        spawn_points = world.get_map().get_spawn_points()
        vehicle = world.try_spawn_actor(blueprint, spawn_points[0])
        if vehicle is None:
            print('ERROR: spawn failed at spawn point 0.')
            return 1
        if args.sync:
            world.tick()
        else:
            world.wait_for_tick()

        # Let the vehicle settle on the ground first.
        for _ in range(40):
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

        location_start = vehicle.get_location()
        print('Spawned at %s' % location_start)

        # Dump the runtime physics configuration. Zero mass / zero torque /
        # zero wheel radius are the typical reasons a Chaos vehicle never
        # reacts to throttle.
        physics_control = vehicle.get_physics_control()
        print('-' * 50)
        print('Physics control:')
        print('  mass:              %s kg' % physics_control.mass)
        print('  drag coefficient:  %s' % physics_control.drag_coefficient)
        print('  max rpm:           %s' % physics_control.max_rpm)
        print('  idle rpm:          %s' % physics_control.idle_rpm)
        print('  max torque:        %s Nm' % physics_control.max_torque)
        torque_points = list(physics_control.torque_curve)
        print('  torque curve points: %d' % len(torque_points))
        curve_max = max((p.y for p in torque_points), default=0.0)
        print('  torque curve max:    %s Nm' % curve_max)
        if torque_points:
            print('  torque curve: %s' % ['(%.0f rpm, %.1f Nm)' % (p.x, p.y)
                                          for p in torque_points])
        print('  use_automatic_gears: %s' % physics_control.use_automatic_gears)
        try:
            print('  forward gear ratios: %s' % list(physics_control.forward_gear_ratios))
        except TypeError:
            print('  forward gear ratios: <not exposed to python>')
        try:
            print('  final ratio:         %s' % physics_control.final_ratio)
        except Exception:
            pass
        try:
            print('  center of mass:      %s' % physics_control.center_of_mass)
        except Exception:
            pass
        print('  wheels: %d' % len(physics_control.wheels))
        for index, wheel in enumerate(physics_control.wheels):
            try:
                print('  wheel %d: radius=%.2f cm width=%.2f cm mass=%.2f kg '
                      'steer=%.1f deg by_engine=%s offset=%s' % (
                          index, wheel.wheel_radius, wheel.wheel_width,
                          wheel.wheel_mass, wheel.max_steer_angle,
                          wheel.affected_by_engine, wheel.offset))
            except Exception:
                print('  wheel %d attrs: %s' % (index, [
                    attr for attr in dir(wheel) if not attr.startswith('_')]))
        print('-' * 50)
        if physics_control.mass <= 0.0 or curve_max <= 0.0:
            print('WARNING: mass or torque is zero -> vehicle can never move.')

        print('Apply full throttle for %.1f s ...' % args.seconds)

        control = carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0)
        vehicle.apply_control(control)

        t_end = time.time() + args.seconds
        while time.time() < t_end:
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
            velocity = vehicle.get_velocity()
            speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5
            print('  speed: %6.1f km/h  acceleration: %s'
                  % (speed, vehicle.get_acceleration()))

        location_end = vehicle.get_location()
        displacement = location_start.distance(location_end)
        final_speed = 3.6 * sum(v**2 for v in (
            vehicle.get_velocity().x,
            vehicle.get_velocity().y,
            vehicle.get_velocity().z)) ** 0.5

        print('-' * 50)
        print('Displacement: %.2f m | Final speed: %.1f km/h'
              % (displacement, final_speed))
        if displacement > 1.0:
            print('RESULT: vehicle MOVES. The problem is in the agent/script layer.')
        else:
            print('RESULT: vehicle does NOT move. The problem is the vehicle')
            print('physics configuration (BP_East_LUV_3151): check wheel setups,')
            print('torque curve, engine max RPM and mass.')
        return 0
    finally:
        if vehicle is not None:
            vehicle.destroy()
        if args.sync:
            original_settings.synchronous_mode = False
            original_settings.fixed_delta_seconds = None
            world.apply_settings(original_settings)


if __name__ == '__main__':
    sys.exit(main())
