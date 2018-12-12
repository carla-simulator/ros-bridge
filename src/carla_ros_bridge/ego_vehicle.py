#!/usr/bin/env python

#
# Copyright (c) 2018 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
"""
Classes to handle Carla vehicles
"""
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA

from carla import VehicleControl
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.msg import CarlaVehicleControl  # pylint: disable=no-name-in-module,import-error


class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    def __init__(self, carla_actor, parent):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        """

        super(EgoVehicle, self).__init__(carla_actor=carla_actor,
                                         parent=parent,
                                         topic_prefix="ego_vehicle",
                                         append_role_name_topic_postfix=False)

        self.control_subscriber = rospy.Subscriber(
            self.topic_name() + "/vehicle_control_cmd",
            CarlaVehicleControl, self.control_command_updated)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on AckermannDrive control commands.
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroy EgoVehicle(id={})".format(self.get_id()))
        self.control_subscriber = None
        super(EgoVehicle, self).destroy()

    def get_marker_color(self):
        """
        Function (override) to return the color for marker messages.

        The ego vehicle uses a different marker color than other vehicles.

        :return: the color used by a ego vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 0
        color.g = 255
        color.b = 0
        return color

    def send_object_msg(self):
        """
        Function (override) to send odometry message of the ego vehicle
        instead of an object message.

        The ego vehicle doesn't send its information as part of the object list.
        A nav_msgs.msg.Odometry is prepared to be published via '/carla/ego_vehicle'

        :return:
        """
        odometry = Odometry(header=self.get_msg_header())
        odometry.child_frame_id = self.get_frame_id()

        odometry.pose.pose = self.get_current_ros_pose()
        odometry.twist.twist = self.get_current_ros_twist()

        self.publish_ros_message(self.topic_name() + "/odometry", odometry)

    def control_command_updated(self, ros_vehicle_control):
        """
        Receive a CarlaVehicleControl msg and send to CARLA

        This function gets called whenever a ROS message is received via
        '/carla/ego_vehicle/vehicle_control_cmd' topic.
        The received ROS message is converted into carla.VehicleControl command and
        sent to CARLA.
        This brigde is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param ros_vehicle_control:
        :type ros_vehicle_control: carla_ros_bridge.msg.CarlaVehicleControl
        :return:
        """
        vehicle_control = VehicleControl()
        vehicle_control.hand_brake = ros_vehicle_control.hand_brake
        vehicle_control.brake = ros_vehicle_control.brake
        vehicle_control.steer = ros_vehicle_control.steer
        vehicle_control.throttle = ros_vehicle_control.throttle
        vehicle_control.reverse = ros_vehicle_control.reverse

        self.carla_actor.apply_control(vehicle_control)
