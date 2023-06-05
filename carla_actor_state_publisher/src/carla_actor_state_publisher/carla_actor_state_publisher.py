#!/usr/bin/env python
#
# Copyright (c) 2023 Valentin Rusche
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Publishes current state information about the actor vehicle as a service
"""
import math
import carla
import numpy

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Header
from carla_msgs.msg import CarlaWorldInfo, CarlaStatus, CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel
from carla_actor_state_types.srv import GetActorState

from carla_ros_bridge.ego_vehicle import EgoVehicle


class CarlaActorStatePublisher(CompatibleNode):

    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """

    def __init__(self):
        """
        Constructor
        """
        super(CarlaActorStatePublisher, self).__init__(
            'carla_actor_state_publisher')
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.role_name = self.get_param("role_name", 'ego_vehicle')
        self.current_tick = 0.0

        # initialize ros service
        self.get_waypoint_service = self.new_service(
            GetActorState,
            '/carla_actor_state_publisher/{}/get_actor_state'.format(self.role_name),
            self.get_actor_state)

        # use callback to wait for ego vehicle
        self.loginfo("Waiting for ego vehicle...")
        self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)

        self.carla_status = CarlaStatus()
        self.status_subscriber = self.new_subscription(
            CarlaStatus,
            "/carla/status",
            self.carla_status_updated,
            qos_profile=10)

    def destroy(self):
        """
        Destructor
        """
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)

    def get_actor_state(self, req, response=None):
        """
        Convenience method to get the waypoint for an actor
        """
        actor = self.world.get_actors().find(req.id)

        response = roscomp.get_service_response(GetActorState)
        if actor:
            response.state.current_tick = self.current_tick

            response.state.is_ego_vehicle = actor.attributes.get(
                'role_name') == self.role_name

            simulator_waypoint = self.map.get_waypoint(actor.get_location())
            self.append_waypoint_data(waypoint = simulator_waypoint, response = response)

            vehicle_status = self.build_actor_status(actor = actor)

            response.ego_vehicle_status = vehicle_status

            vehicle_info = self.build_actor_info(actor = actor)

            response.ego_vehicle_info = vehicle_info
        else:
            self.logwarn(
                "get_actor_state(): Actor {} not valid.".format(req.id))
        return response

    def connect_to_carla(self):

        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(
                    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")

    def carla_status_updated(self, data):
        """
        Callback on carla status
        """
        self.carla_status = data
        if self.carla_status.fixed_delta_seconds:
            self.current_tick = float(
                self.carla_status.fixed_delta_seconds) * float(self.carla_status.frame)
        else:
            self.current_tick = (1.0 / 20.0) * float(self.carla_status.frame)

    def append_waypoint_data(self, waypoint, response):
        response.state.current_waypoint.pose = trans.carla_transform_to_ros_pose(
            waypoint.transform)
        response.state.current_waypoint.is_junction = waypoint.is_junction
        response.state.current_waypoint.road_id = waypoint.road_id
        response.state.current_waypoint.section_id = waypoint.section_id
        response.state.current_waypoint.lane_id = waypoint.lane_id

    def build_actor_status(self, actor):
        header = Header()
        header.frame_id = self.current_tick
        timestamp = self.get_time()
        header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)

        vehicle_status = CarlaEgoVehicleStatus(header=header)
        vehicle_status.velocity = math.sqrt(
            EgoVehicle.get_vehicle_speed_squared(actor))
        vehicle_status.acceleration.linear = trans.carla_acceleration_to_ros_accel(
        actor.get_acceleration()).linear
        vehicle_status.orientation = trans.carla_transform_to_ros_pose(
            actor.get_transform()).orientation
        vehicle_status.control.throttle = actor.get_control().throttle
        vehicle_status.control.steer = actor.get_control().steer
        vehicle_status.control.brake = actor.get_control().brake
        vehicle_status.control.hand_brake = actor.get_control().hand_brake
        vehicle_status.control.reverse = actor.get_control().reverse
        vehicle_status.control.gear = actor.get_control().gear
        vehicle_status.control.manual_gear_shift = actor.get_control().manual_gear_shift
        return vehicle_status
    
    def build_actor_info(self, actor):
        vehicle_info = CarlaEgoVehicleInfo()
        vehicle_info.id = actor.id
        vehicle_info.type = actor.type_id
        vehicle_info.rolename = actor.attributes.get('role_name')
        vehicle_physics = actor.get_physics_control()
        for wheel in vehicle_physics.wheels:
            wheel_info = CarlaEgoVehicleInfoWheel()
            wheel_info.tire_friction = wheel.tire_friction
            wheel_info.damping_rate = wheel.damping_rate
            wheel_info.max_steer_angle = math.radians(
                wheel.max_steer_angle)
            wheel_info.radius = wheel.radius
            wheel_info.max_brake_torque = wheel.max_brake_torque
            wheel_info.max_handbrake_torque = wheel.max_handbrake_torque

            inv_T = numpy.array(
                self.carla_actor.get_transform().get_inverse_matrix(), dtype=float)
            wheel_pos_in_map = numpy.array([wheel.position.x/100.0,
                                            wheel.position.y/100.0,
                                            wheel.position.z/100.0,
                                            1.0])
            wheel_pos_in_ego_vehicle = numpy.matmul(
                inv_T, wheel_pos_in_map)
            wheel_info.position.x = wheel_pos_in_ego_vehicle[0]
            wheel_info.position.y = -wheel_pos_in_ego_vehicle[1]
            wheel_info.position.z = wheel_pos_in_ego_vehicle[2]
            vehicle_info.wheels.append(wheel_info)

        vehicle_info.max_rpm = vehicle_physics.max_rpm
        vehicle_info.max_rpm = vehicle_physics.max_rpm
        vehicle_info.moi = vehicle_physics.moi
        vehicle_info.damping_rate_full_throttle = vehicle_physics.damping_rate_full_throttle
        vehicle_info.damping_rate_zero_throttle_clutch_engaged = \
            vehicle_physics.damping_rate_zero_throttle_clutch_engaged
        vehicle_info.damping_rate_zero_throttle_clutch_disengaged = \
            vehicle_physics.damping_rate_zero_throttle_clutch_disengaged
        vehicle_info.use_gear_autobox = vehicle_physics.use_gear_autobox
        vehicle_info.gear_switch_time = vehicle_physics.gear_switch_time
        vehicle_info.clutch_strength = vehicle_physics.clutch_strength
        vehicle_info.mass = vehicle_physics.mass
        vehicle_info.drag_coefficient = vehicle_physics.drag_coefficient
        vehicle_info.center_of_mass.x = vehicle_physics.center_of_mass.x
        vehicle_info.center_of_mass.y = vehicle_physics.center_of_mass.y
        vehicle_info.center_of_mass.z = vehicle_physics.center_of_mass.z
        return vehicle_info


def main(args=None):
    """
    main function
    """
    roscomp.init('carla_actor_state_publisher', args)

    actor_state_publisher = None
    try:
        actor_state_publisher = CarlaActorStatePublisher()
        actor_state_publisher.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if actor_state_publisher:
            actor_state_publisher.destroy()
        roscomp.shutdown()


if __name__ == "__main__":
    main()
