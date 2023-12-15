#!/usr/bin/env python3

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import math
import random
import time



def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, args):
        self.world = carla_world
        self.sync = args.sync
        self.max_vehicles = args.max_vehicles
        self.speed = str(args.speed)
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.player = None
        self.players = []
        self._actor_generation = args.generation
        self.start()
        # self.world.on_tick(hud.on_world_tick)
        self.constant_velocity_enabled = False

    def start(self):

        spawn_points = self.world.get_map().get_spawn_points()
        max_vehicles = self.max_vehicles
        max_vehicles = min([max_vehicles, len(spawn_points)])

        # Get a erp42 blueprint.
        blueprints = []
        traffic = []
        # blueprint = random.choice(get_actor_blueprints(self.world, 'erp42'+self.speed, self._actor_generation))
        blueprint = random.choice(get_actor_blueprints(self.world, 'erp42npc15', self._actor_generation))

        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')

        for i in range(max_vehicles):
            blueprint.set_attribute('role_name', 'npc'+self.speed+str(i))
            blueprints.append(blueprint)

        spawn_points = self.map.get_spawn_points()

        for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
            temp = self.world.try_spawn_actor(blueprints[i], spawn_point)
            self.show_vehicle_telemetry = False
            temp.set_autopilot(True)
            if temp is not None:
                self.players.append(temp)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def destroy(self):
        for i in range(len(self.players)):
            self.players[i].destroy()



def spawn_loop(args):
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        world = World(sim_world, args)
        # controller = KeyboardControl(world, args.autopilot)

        # print(world.get_speed_set_vehicle())
        # traffic_manager.vehicle_percentage_speed_difference(1, -20.0) 

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        while True:
            if args.sync:
                sim_world.tick()
            # clock.tick_busy_loop(60)
            # if controller.parse_events(client, world, clock, args.sync):
                # return
            # world.tick(clock)
            # world.render(display)

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if world is not None:
            world.destroy()

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    argparser.add_argument(
        '--max_vehicles',
        default=55,
        type=int,
        help='max vehicles to spawn (default: 50)')
    argparser.add_argument(
        '--speed',
        default=20,
        type=int,
        help='speed of vehicles to spawn (default: 20)')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)
    try:
        spawn_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':

    main()
