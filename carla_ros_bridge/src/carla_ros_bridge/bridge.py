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


from rosgraph_msgs.msg import Clock
try:
    import queue
except ImportError:
    import Queue as queue

import os
import sys
from distutils.version import LooseVersion
from threading import Thread, Lock, Event
import pkg_resources

from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker  # pylint: disable=import-error

from ros_compatibility import CompatibleNode, ros_ok, destroy_subscription, ros_shutdown, ros_timestamp, QoSProfile, latch_on
import carla


from carla_ros_bridge.actor import Actor
from carla_ros_bridge.sensor import Sensor

from carla_ros_bridge.carla_status_publisher import CarlaStatusPublisher
from carla_ros_bridge.world_info import WorldInfo
from carla_ros_bridge.spectator import Spectator
from carla_ros_bridge.traffic import Traffic, TrafficLight
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.lidar import Lidar
from carla_ros_bridge.radar import Radar
from carla_ros_bridge.gnss import Gnss
from carla_ros_bridge.imu import ImuSensor
from carla_ros_bridge.ego_vehicle import EgoVehicle
from carla_ros_bridge.collision_sensor import CollisionSensor
from carla_ros_bridge.lane_invasion_sensor import LaneInvasionSensor
from carla_ros_bridge.camera import Camera, RgbCamera, DepthCamera, SemanticSegmentationCamera
from carla_ros_bridge.object_sensor import ObjectSensor
from carla_ros_bridge.rss_sensor import RssSensor
from carla_ros_bridge.walker import Walker
from carla_ros_bridge.debug_helper import DebugHelper
from carla_ros_bridge.traffic_lights_sensor import TrafficLightsSensor
from carla_msgs.msg import CarlaActorList, CarlaActorInfo, CarlaControl, CarlaWeatherParameters  # pylint: disable=import-error


ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    import rospy  # pylint: disable=import-error
elif ROS_VERSION == 2:
    import rclpy  # pylint: disable=import-error
    from rclpy.callback_groups import ReentrantCallbackGroup  # pylint: disable=import-error
    from builtin_interfaces.msg import Time
else:
    raise NotImplementedError("Make sure you have a valid ROS_VERSION env variable set.")


class CarlaRosBridge(CompatibleNode):
    """
    Carla Ros bridge
    """

    CARLA_VERSION = "0.9.9"

    def __init__(self, rospy_init=True, executor=None):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        self.debug_helper = None
        self.carla_weather_subscriber = None
        self.update_lock = Lock()
        self.carla_control_queue = queue.Queue()
        self.on_tick_id = None
        self.update_actor_thread = None
        self.carla_settings = None
        self.shutdown = None
        self.actors = {}
        self.pseudo_actors = []
        super(CarlaRosBridge, self).__init__("ros_bridge_node", rospy_init=rospy_init)
        self.executor = executor

    # pylint: disable=attribute-defined-outside-init
    def initialize_bridge(self, carla_world, params):
        """
        Initialize the bridge
        """
        self.parameters = params
        self.carla_world = carla_world

        if ROS_VERSION == 1:
            self.ros_timestamp = 0
            self.callback_group = None
        elif ROS_VERSION == 2:
            self.ros_timestamp = Time()
            self.callback_group = ReentrantCallbackGroup()

        self.synchronous_mode_update_thread = None
        self.shutdown = Event()
        # set carla world settings
        self.carla_settings = carla_world.get_settings()

        # workaround: settings can only applied within non-sync mode
        if self.carla_settings.synchronous_mode:
            self.carla_settings.synchronous_mode = False
            carla_world.apply_settings(self.carla_settings)

        self.loginfo("Parameters:")
        for key in self.parameters:
            self.loginfo("  {}: {}".format(key, self.parameters[key]))
        self.carla_settings.synchronous_mode = self.parameters["synchronous_mode"]
        self.carla_settings.fixed_delta_seconds = self.parameters["fixed_delta_seconds"]
        carla_world.apply_settings(self.carla_settings)

        self.marker_publisher = self.new_publisher(Marker, '/carla/marker')
        self.tf_publisher = self.new_publisher(TFMessage, 'tf')
        if ROS_VERSION == 1:
            self.clock_publisher = self.new_publisher(Clock, 'clock')
        elif ROS_VERSION == 2:
            self.clock_publisher = self.new_publisher(Clock, 'clock')
        self.actor_list_publisher = self.new_publisher(
            CarlaActorList, "/carla/actor_list", qos_profile=QoSProfile(depth=10, durability=latch_on))
        self.status_publisher = CarlaStatusPublisher(self.carla_settings.synchronous_mode,
                                                     self.carla_settings.fixed_delta_seconds,
                                                     self)

        # for waiting for ego vehicle control commands in synchronous mode,
        # their ids are maintained in a list.
        # Before tick(), the list is filled and the loop waits until the list is empty.
        self._all_vehicle_control_commands_received = Event()
        self._expected_ego_vehicle_control_command_ids = []
        self._expected_ego_vehicle_control_command_ids_lock = Lock()

        if self.carla_settings.synchronous_mode:
            self.carla_run_state = CarlaControl.PLAY

            self.carla_control_subscriber = \
                self.create_subscriber(CarlaControl, "/carla/control",
                                       lambda control: self.carla_control_queue.put(
                                           control.command), callback_group=self.callback_group)

            self.synchronous_mode_update_thread = Thread(target=self._synchronous_mode_update)
            self.synchronous_mode_update_thread.start()
        else:
            self.timestamp_last_run = 0.0

            self.update_actors_queue = queue.Queue(maxsize=1)

            # start thread to update actors
            self.update_actor_thread = Thread(target=self._update_actors_thread)
            self.update_actor_thread.start()

            # create initially existing actors
            self.update_actors_queue.put(set([x.id for x in self.carla_world.get_snapshot()]))

            # wait for first actors creation to be finished
            self.update_actors_queue.join()

            # register callback to update actors
            self.on_tick_id = self.carla_world.on_tick(self._carla_time_tick)

        self.carla_weather_subscriber = \
            self.create_subscriber(CarlaWeatherParameters, "/carla/weather_control",
                                   self.on_weather_changed, callback_group=self.callback_group)

        # add world info
        self.pseudo_actors.append(WorldInfo(carla_world=self.carla_world, node=self))

        # add global object sensor
        self.pseudo_actors.append(
            ObjectSensor(parent=None, node=self, actor_list=self.actors,
                         filtered_id=None))
        self.debug_helper = DebugHelper(carla_world.debug, self)

        # add traffic light pseudo sensor
        self.pseudo_actors.append(
            TrafficLightsSensor(parent=None, node=self, actor_list=self.actors))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        if self.debug_helper:
            self.debug_helper.destroy()
        if self.shutdown:
            self.shutdown.set()
        self.carla_control_queue.put(CarlaControl.STEP_ONCE)
        if self.carla_settings:
            if not self.carla_settings.synchronous_mode:
                if self.on_tick_id:
                    self.carla_world.remove_on_tick(self.on_tick_id)
                if self.update_actor_thread:
                    self.update_actor_thread.join()
        self._update_actors(set())

        self.loginfo("Exiting Bridge")
        super(CarlaRosBridge, self).destroy()

    def on_weather_changed(self, weather_parameters):
        """
        Callback on new weather parameters
        :return:
        """
        if not self.carla_world:
            return
        self.loginfo("Applying weather parameters...")
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

        while command is not None and ros_ok():
            self.carla_run_state = command

            if self.carla_run_state == CarlaControl.PAUSE:
                # wait for next command
                self.loginfo("State set to PAUSED")
                self.status_publisher.set_synchronous_mode_running(False)
                command = self.carla_control_queue.get()
            elif self.carla_run_state == CarlaControl.PLAY:
                self.loginfo("State set to PLAY")
                self.status_publisher.set_synchronous_mode_running(True)
                return
            elif self.carla_run_state == CarlaControl.STEP_ONCE:
                self.loginfo("Execute single step.")
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
                    for actor_id, actor in self.actors.items():
                        if isinstance(actor, EgoVehicle):
                            self._expected_ego_vehicle_control_command_ids.append(actor_id)

            frame = self.carla_world.tick()
            world_snapshot = self.carla_world.get_snapshot()

            self.status_publisher.set_frame(frame)
            self.update_clock(world_snapshot.timestamp)
            self.logdebug("Tick for frame {} returned. Waiting for sensor data...".format(frame))
            self._update(frame, world_snapshot.timestamp.elapsed_seconds)
            self.logdebug("Waiting for sensor data finished.")
            self._update_actors(set([x.id for x in world_snapshot]))

            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                # wait for all ego vehicles to send a vehicle control command
                if self._expected_ego_vehicle_control_command_ids:
                    if not self._all_vehicle_control_commands_received.wait(1):
                        self.logwarn("Timeout (1s) while waiting for vehicle control commands. "
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
            if self.update_lock.acquire(False):
                if self.timestamp_last_run < carla_snapshot.timestamp.elapsed_seconds:
                    self.timestamp_last_run = carla_snapshot.timestamp.elapsed_seconds
                    self.update_clock(carla_snapshot.timestamp)
                    self.status_publisher.set_frame(carla_snapshot.frame)

                    self._update(carla_snapshot.frame, carla_snapshot.timestamp.elapsed_seconds)
                self.update_lock.release()

            # if possible push current snapshot to update-actors-thread
            try:
                self.update_actors_queue.put_nowait(set([x.id for x in carla_snapshot]))
            except queue.Full:
                pass

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.ros_timestamp = ros_timestamp(carla_timestamp.elapsed_seconds, from_sec=True)
        if ROS_VERSION == 1:
            self.clock_publisher.publish(Clock(self.ros_timestamp))
        elif ROS_VERSION == 2:
            self.clock_publisher.publish(Clock(clock=self.ros_timestamp))

    def _update_actors_thread(self):
        """
        execution loop for async mode actor list updates
        """
        while not self.shutdown.is_set():
            try:
                current_actors = self.update_actors_queue.get(timeout=1)
                if current_actors:
                    self._update_actors(current_actors)
                    self.update_actors_queue.task_done()
            except queue.Empty:
                pass

    def _update_actors(self, current_actors):
        """
        update the available actors
        """
        previous_actors = set(self.actors)

        new_actors = current_actors - previous_actors
        deleted_actors = previous_actors - current_actors

        if new_actors:
            for carla_actor in self.carla_world.get_actors(list(new_actors)):
                self._create_actor(carla_actor)

        if deleted_actors:
            for id_to_delete in deleted_actors:
                # remove actor
                actor = self.actors[id_to_delete]
                with self.update_lock:
                    self.loginfo("Remove {}(id={}, parent_id={}, prefix={})".format(
                        actor.__class__.__name__, actor.get_id(), actor.get_parent_id(),
                        actor.get_prefix()))
                    actor.destroy()
                    del self.actors[id_to_delete]

                # remove pseudo-actors that have actor as parent
                updated_pseudo_actors = []
                for pseudo_actor in self.pseudo_actors:
                    if pseudo_actor.get_parent_id() == id_to_delete:
                        self.loginfo("Remove {}(parent_id={}, prefix={})".format(
                            pseudo_actor.__class__.__name__, pseudo_actor.get_parent_id(),
                            pseudo_actor.get_prefix()))
                        pseudo_actor.destroy()
                        del pseudo_actor
                    else:
                        updated_pseudo_actors.append(pseudo_actor)
                self.pseudo_actors = updated_pseudo_actors

        # publish actor list on change
        if new_actors or deleted_actors:
            self.publish_actor_list()

    def publish_actor_list(self):
        """
        publish list of carla actors
        :return:
        """
        ros_actor_list = CarlaActorList()

        for actor_id in self.actors:
            actor = self.actors[actor_id].carla_actor
            ros_actor = CarlaActorInfo()
            ros_actor.id = actor.id
            ros_actor.type = actor.type_id
            try:
                ros_actor.rolename = str(actor.attributes.get('role_name'))
            except ValueError:
                pass

            if actor.parent:
                ros_actor.parent_id = actor.parent.id
            else:
                ros_actor.parent_id = 0

            ros_actor_list.actors.append(ros_actor)

        self.actor_list_publisher.publish(ros_actor_list)

    def _create_actor(self, carla_actor):  # pylint: disable=too-many-branches,too-many-statements
        """
        create an actor
        """
        parent = None
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self._create_actor(carla_actor.parent)

        actor = None
        pseudo_actors = []
        if carla_actor.type_id.startswith('traffic'):
            if carla_actor.type_id == "traffic.traffic_light":
                actor = TrafficLight(carla_actor, parent, node=self)
            else:
                actor = Traffic(carla_actor, parent, node=self)
        elif carla_actor.type_id.startswith("vehicle"):
            if carla_actor.attributes.get('role_name') \
                    in self.parameters['ego_vehicle']['role_name']:
                actor = EgoVehicle(carla_actor, parent, self,
                                   self._ego_vehicle_control_applied_callback)
                pseudo_actors.append(
                    ObjectSensor(parent=actor, node=self, actor_list=self.actors,
                                 filtered_id=carla_actor.id))
            else:
                actor = Vehicle(carla_actor, parent, self)
        elif carla_actor.type_id.startswith("sensor"):
            if carla_actor.type_id.startswith("sensor.camera"):
                if carla_actor.type_id.startswith("sensor.camera.rgb"):
                    actor = RgbCamera(carla_actor, parent, self,
                                      self.carla_settings.synchronous_mode)
                elif carla_actor.type_id.startswith("sensor.camera.depth"):
                    actor = DepthCamera(carla_actor, parent, self,
                                        self.carla_settings.synchronous_mode)
                elif carla_actor.type_id.startswith("sensor.camera.semantic_segmentation"):
                    actor = SemanticSegmentationCamera(carla_actor, parent, self,
                                                       self.carla_settings.synchronous_mode)
                else:
                    actor = Camera(carla_actor, parent, self,
                                   self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.lidar"):
                actor = Lidar(carla_actor, parent, self, self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.other.radar"):
                actor = Radar(carla_actor, parent, self, self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.other.gnss"):
                actor = Gnss(carla_actor, parent, self, self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.other.imu"):
                actor = ImuSensor(carla_actor, parent, self,
                                  self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.other.collision"):
                actor = CollisionSensor(
                    carla_actor, parent, self, self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.other.rss"):
                actor = RssSensor(
                    carla_actor, parent, self, self.carla_settings.synchronous_mode)
            elif carla_actor.type_id.startswith("sensor.other.lane_invasion"):
                actor = LaneInvasionSensor(carla_actor, parent, self,
                                           self.carla_settings.synchronous_mode)
            else:
                actor = Sensor(carla_actor, parent, self,
                               self.carla_settings.synchronous_mode)
        elif carla_actor.type_id.startswith("spectator"):
            actor = Spectator(carla_actor, parent, node=self)
        elif carla_actor.type_id.startswith("walker"):
            actor = Walker(carla_actor, parent, node=self)
        else:
            actor = Actor(carla_actor, parent, node=self)

        self.loginfo("Created {}(id={}, parent_id={},"
                     " type={}, prefix={}, attributes={})".format(actor.__class__.__name__,
                                                                  actor.get_id(),
                                                                  actor.get_parent_id(),
                                                                  carla_actor.type_id,
                                                                  actor.get_prefix(),
                                                                  carla_actor.attributes))
        with self.update_lock:
            self.actors[carla_actor.id] = actor

        for pseudo_actor in pseudo_actors:
            self.loginfo("Created {}(parent_id={}, prefix={})".format(
                pseudo_actor.__class__.__name__, pseudo_actor.get_parent_id(),
                pseudo_actor.get_prefix()))
            with self.update_lock:
                self.pseudo_actors.append(pseudo_actor)

        return actor

    def _update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        # update all pseudo actors
        for actor in self.pseudo_actors:
            actor.update(frame_id, timestamp)

        # update all carla actors
        for actor_id in self.actors:
            try:
                self.actors[actor_id].update(frame_id, timestamp)
            except RuntimeError as e:
                self.logwarn("Update actor {}({}) failed: {}".format(
                    self.actors[actor_id].__class__.__name__, actor_id, e))
                continue

    def _ego_vehicle_control_applied_callback(self, ego_vehicle_id):
        if not self.carla_settings.synchronous_mode or \
                not self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
            return
        with self._expected_ego_vehicle_control_command_ids_lock:
            if ego_vehicle_id in self._expected_ego_vehicle_control_command_ids:
                self._expected_ego_vehicle_control_command_ids.remove(ego_vehicle_id)
            else:
                self.logwarn(
                    "Unexpected vehicle control command received from {}".format(ego_vehicle_id))
            if not self._expected_ego_vehicle_control_command_ids:
                self._all_vehicle_control_commands_received.set()

    def publish_tf_message(self, msg):
        """
        Function to publish ROS TF message.
        """

        # prepare tf message
        tf_msg = None
        tf_to_publish = []
        tf_to_publish.append(msg)
        if ROS_VERSION == 1:
            tf_msg = TFMessage(tf_to_publish)
        elif ROS_VERSION == 2:
            tf_msg = TFMessage()
            tf_msg.transforms = tf_to_publish
        try:
            self.tf_publisher.publish(tf_msg)
        except Exception as error:  # pylint: disable=broad-except
            self.logwarn("Failed to publish message: {}".format(error))


def main():
    """
    main function for carla simulator ROS bridge
    maintaining the communication client and the CarlaBridge object
    """
    carla_bridge = None
    carla_world = None
    carla_client = None
    executor = None
    parameters = {}
    if ROS_VERSION == 1:
        carla_bridge = CarlaRosBridge()
        # rospy.init_node('carla_ros_bridge', anonymous=True)

    elif ROS_VERSION == 2:
        rclpy.init(args=None)
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=12)
        carla_bridge = CarlaRosBridge(executor=executor)
        # init_node = rclpy.create_node("init_ros_bridge")
        executor.add_node(carla_bridge)

    parameters['host'] = carla_bridge.get_param('host', 'localhost')
    parameters['port'] = carla_bridge.get_param('port', 2000)
    parameters['timeout'] = carla_bridge.get_param('timeout', 2)
    parameters['synchronous_mode'] = carla_bridge.get_param('synchronous_mode', False)
    parameters['synchronous_mode_wait_for_vehicle_control_command'] = carla_bridge.get_param(
        'synchronous_mode_wait_for_vehicle_control_command', False)
    parameters['fixed_delta_seconds'] = carla_bridge.get_param('fixed_delta_seconds',
                                                               0.05)
    parameters['town'] = carla_bridge.get_param('town', 'Town01')
    role_name = carla_bridge.get_param('ego_vehicle_role_name',
                                       ["hero", "ego_vehicle", "hero1", "hero2", "hero3"])
    parameters["ego_vehicle"] = {"role_name": role_name}

    carla_bridge.loginfo("Trying to connect to {host}:{port}".format(
        host=parameters['host'], port=parameters['port']))

    try:
        carla_client = carla.Client(host=parameters['host'], port=parameters['port'])
        carla_client.set_timeout(parameters['timeout'])

        # check carla version
        dist = pkg_resources.get_distribution("carla")
        if LooseVersion(dist.version) < LooseVersion(CarlaRosBridge.CARLA_VERSION):
            raise RuntimeError("CARLA Server version {} required. Found: {}".format(
                CarlaRosBridge.CARLA_VERSION, carla_client.get_server_version()))

        if LooseVersion(carla_client.get_server_version()) < \
                LooseVersion(CarlaRosBridge.CARLA_VERSION):
            raise RuntimeError("CARLA Server version {} required. Found: {}".format(
                CarlaRosBridge.CARLA_VERSION, carla_client.get_server_version()))

        carla_world = carla_client.get_world()

        if "town" in parameters:
            if parameters["town"].endswith(".xodr"):
                carla_bridge.loginfo("Loading opendrive world from file '{}'".format(
                    parameters["town"]))
                with open(parameters["town"]) as od_file:
                    data = od_file.read()
                carla_world = carla_client.generate_opendrive_world(str(data))
            else:
                if carla_world.get_map().name != parameters["town"]:
                    carla_bridge.loginfo("Loading town '{}' (previous: '{}').".format(
                        parameters["town"],
                        carla_world.get_map().name))
                    carla_world = carla_client.load_world(parameters["town"])
            carla_world.tick()

        carla_bridge.initialize_bridge(carla_client.get_world(), parameters)

        if ROS_VERSION == 1:
            rospy.spin()
        elif ROS_VERSION == 2:
            executor.spin()
    except (IOError, RuntimeError) as e:
        carla_bridge.logerr("Error: {}".format(e))
    except KeyboardInterrupt:
        pass
    finally:
        carla_bridge.destroy()
        ros_shutdown()
        del carla_world
        del carla_client


if __name__ == "__main__":
    main()
