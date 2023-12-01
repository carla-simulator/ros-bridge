#!/usr/bin/env python3

import carla

from agents.navigation.global_route_planner import GlobalRoutePlanner
# from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO





def main():

    client = carla.Client("localhost", 2000)
    client.set_timeout(10)

    world = client.get_world()

    amap = world.get_map()
    sampling_resolution = 2
    grp = GlobalRoutePlanner(amap, sampling_resolution)
    # grp = GlobalRoutePlanner(dao)
    # dao.setup()
    spawn_points = amap.get_spawn_points()
    a = carla.Location(spawn_points[0].location)
    b = carla.Location(spawn_points[1].location)

    w1 = grp.trace_route(a, b) # there are other funcations can be used to gener
    # ate a route in GlobalRoutePlanner.
    for w in w1:
        world.debug.draw_string(w[0].transform.location, 'O', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=20.0,
        persistent_lines=True)
        print(w)

    print('\nBye!')


if __name__ == '__main__':

    main()

