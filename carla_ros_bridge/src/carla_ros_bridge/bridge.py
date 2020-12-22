#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosbridge class:

Class that handle communication between CARLA and ROS
"""
try:
    import queue
except ImportError:
    import Queue as queue

import sys
from distutils.version import LooseVersion
from threading import Thread, Lock, Event
import pkg_resources
import rospy

import random

from rosgraph_msgs.msg import Clock

import carla

import carla_common.transforms as trans

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.actor_factory import ActorFactory

from carla_ros_bridge.carla_status_publisher import CarlaStatusPublisher
from carla_ros_bridge.debug_helper import DebugHelper
from carla_ros_bridge.world_info import WorldInfo
from carla_ros_bridge.ego_vehicle import EgoVehicle

from carla_msgs.msg import CarlaControl, CarlaWeatherParameters
from carla_msgs.srv import SpawnObject, SpawnObjectResponse, DestroyObject, DestroyObjectResponse, GetBlueprints, GetBlueprintsResponse


# to generate a random spawning position or vehicles
secure_random = random.SystemRandom()


class CarlaRosBridge(object):

    """
    Carla Ros bridge
    """

    CARLA_VERSION = "0.9.10"

    def __init__(self, carla_world, params):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        self.parameters = params
        self.carla_world = carla_world

        self.synchronous_mode_update_thread = None
        self.shutdown = Event()

        self.carla_settings = carla_world.get_settings()
        if not self.parameters["passive"]:
            # workaround: settings can only applied within non-sync mode
            if self.carla_settings.synchronous_mode:
                self.carla_settings.synchronous_mode = False
                carla_world.apply_settings(self.carla_settings)

            rospy.loginfo("synchronous_mode: {}".format(
                self.parameters["synchronous_mode"]))
            self.carla_settings.synchronous_mode = self.parameters["synchronous_mode"]
            rospy.loginfo("fixed_delta_seconds: {}".format(
                self.parameters["fixed_delta_seconds"]))
            self.carla_settings.fixed_delta_seconds = self.parameters["fixed_delta_seconds"]
            carla_world.apply_settings(self.carla_settings)

        # active sync mode in the ros bridge only if CARLA world is configured in sync mode and
        # passive mode is not enabled.
        self.sync_mode = self.carla_settings.synchronous_mode and not self.parameters["passive"]
        if self.carla_settings.synchronous_mode and self.parameters["passive"]:
            rospy.loginfo(
                "Passive mode is enabled and CARLA world is configured in synchronous mode. This configuration requires another client ticking the CARLA world.")

        self.carla_control_queue = queue.Queue()

        # actor factory
        self.actor_factory = ActorFactory(self, carla_world, self.sync_mode)

        # add world info
        self.world_info = WorldInfo(carla_world=self.carla_world)
        # add debug helper
        self.debug_helper = DebugHelper(carla_world.debug)

        # Communication topics
        self.clock_publisher = rospy.Publisher('clock', Clock, queue_size=10)

        self.status_publisher = CarlaStatusPublisher(
            self.carla_settings.synchronous_mode,
            self.carla_settings.fixed_delta_seconds)

        # for waiting for ego vehicle control commands in synchronous mode,
        # their ids are maintained in a list.
        # Before tick(), the list is filled and the loop waits until the list is empty.
        self._all_vehicle_control_commands_received = Event()
        self._expected_ego_vehicle_control_command_ids = []
        self._expected_ego_vehicle_control_command_ids_lock = Lock()

        if self.sync_mode:
            self.carla_run_state = CarlaControl.PLAY

            self.carla_control_subscriber = \
                rospy.Subscriber("/carla/control", CarlaControl,
                                 lambda control: self.carla_control_queue.put(control.command))

            self.synchronous_mode_update_thread = Thread(
                target=self._synchronous_mode_update)
            self.synchronous_mode_update_thread.start()
        else:
            self.timestamp_last_run = 0.0

            self.actor_factory.start()

            # register callback to update actors
            self.on_tick_id = self.carla_world.on_tick(self._carla_time_tick)

        # services configuration.
        self._registered_actors = []
        self.spawn_object_service = rospy.Service("/carla/spawn_object", SpawnObject,
                                                  self.spawn_object)
        self.destroy_object_service = rospy.Service("/carla/destroy_object", DestroyObject,
                                                    self.destroy_object)

        self.get_blueprints_service = rospy.Service("/carla/get_blueprints", GetBlueprints,
                                                    self.get_blueprints)

        self.carla_weather_subscriber = \
            rospy.Subscriber("/carla/weather_control",
                             CarlaWeatherParameters, self.on_weather_changed)

    def _spawn_actor(self, req):
        if "*" in req.type:
            blueprint = secure_random.choice(
                self.carla_world.get_blueprint_library().filter(req.type))
        else:
            blueprint = self.carla_world.get_blueprint_library().find(req.type)
        blueprint.set_attribute('role_name', req.id)
        for attribute in req.attributes:
            blueprint.set_attribute(attribute.key, attribute.value)
        if req.random_pose is False:
            transform = trans.ros_pose_to_carla_transform(req.transform)
        else:
            # get a random pose
            spawn_points = self.carla_world.get_map().get_spawn_points()
            transform = secure_random.choice(
                spawn_points) if spawn_points else carla.Transform()

        attach_to = None
        if req.attach_to != 0:
            attach_to = self.carla_world.get_actor(req.attach_to)
            if attach_to is None:
                raise IndexError("Parent actor {} not found".format(req.attach_to))

        carla_actor = self.carla_world.spawn_actor(blueprint, transform, attach_to)
        actor = self.actor_factory.create(
            req.type, req.id, req.attach_to, req.transform, carla_actor)
        return actor.uid

    def _spawn_pseudo_actor(self, req):
        actor = self.actor_factory.create(req.type, req.id, req.attach_to, req.transform)
        return actor.uid

    def spawn_object(self, req):
        with self.actor_factory.spawn_lock:
            try:
                if "pseudo" in req.type:
                    id_ = self._spawn_pseudo_actor(req)
                else:
                    id_ = self._spawn_actor(req)

                self._registered_actors.append(id_)
                return SpawnObjectResponse(id_, "")

            except Exception as e:
                rospy.logwarn("Error spawning object '{}: {}".format(req.type, e))
                return SpawnObjectResponse(-1, str(e))

    def _destroy_actor(self, uid):
        if uid not in self.actor_factory.actors:
            return False

        # remove actors that have the actor to be removed as parent.
        for actor in list(self.actor_factory.actors.values()):
            if actor.parent is not None and actor.parent.uid == uid:
                if actor.uid in self._registered_actors:
                    success = self._destroy_actor(actor.uid)
                    if not success:
                        return False

        actor = self.actor_factory.actors[uid]
        if isinstance(actor, Actor):
            actor.carla_actor.destroy()

        self.actor_factory.destroy(uid)
        if uid in self._registered_actors:
            self._registered_actors.remove(uid)

        return True

    def destroy_object(self, req):
        with self.actor_factory.spawn_lock:
            result = self._destroy_actor(req.id)
            return DestroyObjectResponse(result)

    def get_blueprints(self, req):
        response = GetBlueprintsResponse()
        if req.filter:
            bp_filter = req.filter
        else:
            bp_filter = "*"

        response.blueprints = [
            bp.id for bp in self.carla_world.get_blueprint_library().filter(bp_filter)]
        response.blueprints.extend(self.actor_factory.get_pseudo_sensor_types())
        response.blueprints.sort()
        return response

    def on_weather_changed(self, weather_parameters):
        """
        Callback on new weather parameters
        :return:
        """
        if not self.carla_world:
            return
        rospy.loginfo("Applying weather parameters...")
        weather = carla.WeatherParameters()
        weather.cloudiness = weather_parameters.cloudiness
        weather.precipitation = weather_parameters.precipitation
        weather.precipitation_deposits = weather_parameters.precipitation_deposits
        weather.wind_intensity = weather_parameters.wind_intensity
        weather.fog_density = weather_parameters.fog_density
        weather.fog_distance = weather_parameters.fog_distance
        weather.wetness = weather_parameters.wetness
        weather.sun_azimuth_angle = weather_parameters.sun_azimuth_angle
        weather.sun_altitude_angle = weather_parameters.sun_altitude_angle
        self.carla_world.set_weather(weather)

    def process_run_state(self):
        """
        process state changes
        """
        command = None

        # get last command
        while not self.carla_control_queue.empty():
            command = self.carla_control_queue.get()

        while command is not None and not rospy.is_shutdown():
            self.carla_run_state = command

            if self.carla_run_state == CarlaControl.PAUSE:
                # wait for next command
                rospy.loginfo("State set to PAUSED")
                self.status_publisher.set_synchronous_mode_running(False)
                command = self.carla_control_queue.get()
            elif self.carla_run_state == CarlaControl.PLAY:
                rospy.loginfo("State set to PLAY")
                self.status_publisher.set_synchronous_mode_running(True)
                return
            elif self.carla_run_state == CarlaControl.STEP_ONCE:
                rospy.loginfo("Execute single step.")
                self.status_publisher.set_synchronous_mode_running(True)
                self.carla_control_queue.put(CarlaControl.PAUSE)
                return

    def _synchronous_mode_update(self):
        """
        execution loop for synchronous mode
        """
        while not self.shutdown.is_set():
            self.process_run_state()

            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                # fill list of available ego vehicles
                self._expected_ego_vehicle_control_command_ids = []
                with self._expected_ego_vehicle_control_command_ids_lock:
                    for actor_id, actor in self.actor_factory.actors.items():
                        if isinstance(actor, EgoVehicle):
                            self._expected_ego_vehicle_control_command_ids.append(
                                actor_id)

            frame = self.carla_world.tick()
            world_snapshot = self.carla_world.get_snapshot()

            self.status_publisher.set_frame(frame)
            self.update_clock(world_snapshot.timestamp)
            rospy.logdebug("Tick for frame {} returned. Waiting for sensor data...".format(
                frame))
            with self.actor_factory.lock:
                self._update(frame, world_snapshot.timestamp.elapsed_seconds)
            rospy.logdebug("Waiting for sensor data finished.")
            self.actor_factory.update()

            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                # wait for all ego vehicles to send a vehicle control command
                if self._expected_ego_vehicle_control_command_ids:
                    if not self._all_vehicle_control_commands_received.wait(1):
                        rospy.logwarn("Timeout (1s) while waiting for vehicle control commands. "
                                      "Missing command from actor ids {}".format(
                                          self._expected_ego_vehicle_control_command_ids))
                    self._all_vehicle_control_commands_received.clear()

    def _carla_time_tick(self, carla_snapshot):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates.

        After successful locking the update mutex
        (only perform trylock to respect bridge processing time)
        the clock and the children are updated.
        Finally the ROS messages collected to be published are sent out.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not self.shutdown.is_set():
            if self.actor_factory.lock.acquire(False):
                if self.timestamp_last_run < carla_snapshot.timestamp.elapsed_seconds:
                    self.timestamp_last_run = carla_snapshot.timestamp.elapsed_seconds
                    self.update_clock(carla_snapshot.timestamp)
                    self.status_publisher.set_frame(carla_snapshot.frame)
                    self._update(carla_snapshot.frame,
                                 carla_snapshot.timestamp.elapsed_seconds)
                self.actor_factory.lock.release()

    def run(self):
        """
        Run the bridge functionality.

        Registers on shutdown callback at rospy and spins ROS.

        :return:
        """
        rospy.on_shutdown(self.on_shutdown)
        rospy.spin()

    def on_shutdown(self):
        """
        Function to be called on shutdown.

        This function is registered at rospy as shutdown handler.

        """
        rospy.loginfo("Shutdown requested")
        self.destroy()

    def _update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        # update world info
        self.world_info.update(frame_id, timestamp)

        # update all carla actors
        for actor_id in self.actor_factory.actors:
            try:
                self.actor_factory.actors[actor_id].update(frame_id, timestamp)
            except RuntimeError as e:
                rospy.logwarn("Update actor {}({}) failed: {}".format(
                    self.actor_factory.actors[actor_id].__class__.__name__, actor_id, e))
                continue

    def _ego_vehicle_control_applied_callback(self, ego_vehicle_id):
        if not self.sync_mode or \
                not self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
            return
        with self._expected_ego_vehicle_control_command_ids_lock:
            if ego_vehicle_id in self._expected_ego_vehicle_control_command_ids:
                self._expected_ego_vehicle_control_command_ids.remove(
                    ego_vehicle_id)
            else:
                rospy.logwarn(
                    "Unexpected vehicle control command received from {}".format(ego_vehicle_id))
            if not self._expected_ego_vehicle_control_command_ids:
                self._all_vehicle_control_commands_received.set()

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.ros_timestamp = rospy.Time.from_sec(
            carla_timestamp.elapsed_seconds)
        self.clock_publisher.publish(Clock(self.ros_timestamp))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        rospy.signal_shutdown("")
        self.debug_helper.destroy()
        self.shutdown.set()
        self.carla_weather_subscriber.unregister()
        self.carla_control_queue.put(CarlaControl.STEP_ONCE)
        if not self.sync_mode:
            if self.on_tick_id:
                self.carla_world.remove_on_tick(self.on_tick_id)
            self.actor_factory.thread.join()

        with self.actor_factory.spawn_lock:
            # remove actors in reverse order to destroy parent actors first.
            for uid in self._registered_actors[::-1]:
                self._destroy_actor(uid)
        self.actor_factory.clear()

        rospy.loginfo("Exiting Bridge")


def main():
    """
    main function for carla simulator ROS bridge
    maintaining the communication client and the CarlaBridge object
    """
    rospy.init_node("carla_bridge", anonymous=True)
    parameters = rospy.get_param('carla')
    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=parameters['host'], port=parameters['port']))

    carla_bridge = None
    carla_world = None
    carla_client = None
    try:
        carla_client = carla.Client(
            host=parameters['host'],
            port=parameters['port'])
        carla_client.set_timeout(parameters['timeout'])

        # check carla version
        dist = pkg_resources.get_distribution("carla")
        if LooseVersion(dist.version) != LooseVersion(CarlaRosBridge.CARLA_VERSION):
            rospy.logfatal("CARLA python module version {} required. Found: {}".format(
                CarlaRosBridge.CARLA_VERSION, dist.version))
            sys.exit(1)

        if LooseVersion(carla_client.get_server_version()) != \
           LooseVersion(carla_client.get_client_version()):
            rospy.logwarn(
                "Version mismatch detected: You are trying to connect to a simulator that might be incompatible with this API. Client API version: {}. Simulator API version: {}"
                .format(carla_client.get_client_version(),
                        carla_client.get_server_version()))

        carla_world = carla_client.get_world()

        if "town" in parameters:
            if parameters["town"].endswith(".xodr"):
                rospy.loginfo(
                    "Loading opendrive world from file '{}'".format(parameters["town"]))
                with open(parameters["town"]) as od_file:
                    data = od_file.read()
                carla_world = carla_client.generate_opendrive_world(str(data))
            else:
                if carla_world.get_map().name != parameters["town"]:
                    rospy.loginfo("Loading town '{}' (previous: '{}').".format(
                        parameters["town"], carla_world.get_map().name))
                    carla_world = carla_client.load_world(parameters["town"])
            carla_world.tick()

        carla_bridge = CarlaRosBridge(carla_client.get_world(), parameters)
        carla_bridge.run()
    except (IOError, RuntimeError) as e:
        rospy.logerr("Error: {}".format(e))
    finally:
        del carla_world
        del carla_client


if __name__ == "__main__":
    main()
