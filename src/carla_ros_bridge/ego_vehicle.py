#!/usr/bin/env python

#
# Copyright (c) 2018 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
"""
Classes to handle Carla vehicles
"""
import sys
import numpy

from simple_pid import PID

import rospy
from dynamic_reconfigure.server import Server

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from ackermann_msgs.msg import AckermannDrive

from carla import VehicleControl

from carla_ros_bridge.vehicle import Vehicle
import carla_ros_bridge.physics as phys
from carla_ros_bridge.msg import CarlaVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error
from carla_ros_bridge.cfg import EgoVehicleControlParameterConfig  # pylint: disable=no-name-in-module,import-error


class EgoVehicle(Vehicle):

    """
    Vehicle implementation details for the ego vehicle
    """

    @staticmethod
    def create_actor(carla_actor, parent):
        """
        Static factory method to create ego vehicle actors

        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        :param parent: the parent of the new traffic actor
        :type parent: carla_ros_bridge.Parent
        :return: the created vehicle actor
        :rtype: carla_ros_bridge.Vehicle or derived type
        """
        ego_vehicle_control_mode = parent.get_param('ego_vehicle').get('control_mode', 'pedal')
        if ego_vehicle_control_mode == 'pedal':
            return PedalControlVehicle(carla_actor=carla_actor, parent=parent)
        elif ego_vehicle_control_mode == 'ackermann':
            return AckermannControlVehicle(carla_actor=carla_actor, parent=parent)
        else:
            raise ValueError(
                "Unsupported control mode of the ego vehicle '{}'"
                " configured at '/carla/ego_vehicle/control_mode' parameter."
                " Only 'pedal' or 'ackermann' allowed!".format(ego_vehicle_control_mode))

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


class PedalControlVehicle(EgoVehicle):

    """
    Vehicle implementation details for the ego vehicle in pedal control variant
    """

    def __init__(self, carla_actor, parent):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        """
        super(PedalControlVehicle, self).__init__(carla_actor=carla_actor,
                                                  parent=parent)

        self.control_subscriber = rospy.Subscriber(
            self.topic_name() + "/vehicle_control_cmd",
            CarlaVehicleControl, self.control_command_updated)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on CarlaVehicleControl commands.
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroy PedalControlVehicle(id={})".format(self.get_id()))
        self.control_subscriber = None
        super(PedalControlVehicle, self).destroy()

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
        # send control command out
        self.carla_actor.apply_control(vehicle_control)


class AckermannControlVehicle(EgoVehicle):

    """
    Vehicle implementation details for the ego vehicle in ackermann control variant
    """

    def __init__(self, carla_actor, parent):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        """
        super(AckermannControlVehicle, self).__init__(carla_actor=carla_actor,
                                                      parent=parent)

        # control info
        self.info = EgoVehicleControlInfo()

        # maximum values
        self.info.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.carla_actor)
        self.info.max_speed = phys.get_vehicle_max_speed(
            self.carla_actor)
        self.info.max_accel = phys.get_vehicle_max_acceleration(
            self.carla_actor)
        self.info.max_decel = phys.get_vehicle_max_deceleration(
            self.carla_actor)

        # target values
        self.info.target_steering_angle = 0.
        self.info.target_speed = 0.
        self.info.target_speed_abs = 0.
        self.info.target_accel_abs = 0.
        self.info.target_decel_abs = 0.

        # current values
        self.info.current_time_sec = self.get_current_ros_time().to_sec()
        self.info.current_speed = 0.
        self.info.current_speed_abs = 0.
        self.info.current_accel = 0.

        # control values
        self.info.status = ''
        self.info.speed_control_accel_delta = 0.
        self.info.speed_control_accel_target = 0.
        self.info.accel_control_pedal_delta = 0.
        self.info.accel_control_pedal_target = 0.
        self.info.brake_upper_border = 0.
        self.info.throttle_lower_border = 0.

        # the carla vehicle control message
        self.vehicle_control = VehicleControl()

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: AckermannControlVehicle.get_current_ros_time(self).to_sec())
        # we might want to use a PID controller to reach the final target speed
        self.speed_controller = PID(Kp=0.0,
                                    Ki=0.0,
                                    Kd=0.0,
                                    sample_time=0.05,
                                    output_limits=(-1., 1.))
        self.accel_controller = PID(Kp=0.0,
                                    Ki=0.0,
                                    Kd=0.0,
                                    sample_time=0.05,
                                    output_limits=(-1, 1))

        self.reconfigure_server = Server(
            EgoVehicleControlParameterConfig,
            namespace=self.topic_name(),
            callback=(lambda config, level: AckermannControlVehicle.reconfigure_pid_parameters(
                self, config, level)))

        # ROS subsriber
        self.control_subscriber = rospy.Subscriber(
            self.topic_name() + "/ackermann_cmd",
            AckermannDrive, self.ackermann_command_updated)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on AckermannDrive commands.
        Finish the PID controllers.
        Destroy the reconfiguration server
        Finally forward call to super class.

        :return:
        """
        rospy.logdebug("Destroy EgoVehicleAckermannControl(id={})".format(self.get_id()))
        self.control_subscriber = None
        self.speed_controller = None
        self.accel_controller = None
        # first cleanup the server (otherwise leaves a mess behind ;-)
        self.reconfigure_server.set_service.shutdown()
        self.reconfigure_server = None
        super(AckermannControlVehicle, self).destroy()

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        super(AckermannControlVehicle, self).update()
        self.vehicle_control_cycle()
        self.send_ego_vehicle_control_info_msg()

    def reconfigure_pid_parameters(self, ego_vehicle_control_parameter, dummy_level):
        """
        Callback for dynamic reconfigure call to set the PID parameters

        :param ego_vehicle_control_parameter:
        :type ego_vehicle_control_parameter: carla_ros_bridge.cfg.EgoVehicleControlParameterConfig

        """
        rospy.loginfo("Reconfigure Request:  "
                      "speed ({speed_Kp}, {speed_Ki}, {speed_Kd}),"
                      "accel ({accel_Kp}, {accel_Ki}, {accel_Kd}),"
                      "".format(**ego_vehicle_control_parameter))
        self.speed_controller.tunings = (
            ego_vehicle_control_parameter['speed_Kp'],
            ego_vehicle_control_parameter['speed_Ki'],
            ego_vehicle_control_parameter['speed_Kd']
        )
        self.accel_controller.tunings = (
            ego_vehicle_control_parameter['accel_Kp'],
            ego_vehicle_control_parameter['accel_Ki'],
            ego_vehicle_control_parameter['accel_Kd']
        )
        return ego_vehicle_control_parameter

    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ros_bridge.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        self.publish_ros_message(
            self.topic_name() + "/ego_vehicle_control_info", self.info)

    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Convert a Ackerman drive msg into carla control msg

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
        # set target values
        self.set_target_steering_angle(ros_ackermann_drive.steering_angle)
        self.set_target_speed(ros_ackermann_drive.speed)
        self.set_target_accel(ros_ackermann_drive.acceleration)

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        if abs(target_steering_angle) > self.info.max_steering_angle:
            rospy.logerr("Max steering angle reached, clipping value")
            self.info.target_steering_angle = numpy.clip(
                target_steering_angle, -self.info.max_steering_angle, self.info.max_steering_angle)
        else:
            self.info.target_steering_angle = target_steering_angle

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.max_speed:
            rospy.logerr("Max speed reached, clipping value")
            self.info.target_speed = numpy.clip(
                target_speed, -self.info.max_speed, self.info.max_speed)
        else:
            self.info.target_speed = target_speed
        self.info.target_speed_abs = abs(self.info.target_speed)

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        epsilon = 0.00001
        # per definition of ackermann drive: if zero, then use max value
        if abs(target_accel) < epsilon:
            self.info.target_accel_abs = self.info.max_accel
            self.info.target_decel_abs = self.info.max_decel
        else:
            self.info.target_accel_abs = numpy.clip(
                abs(target_accel), 0, self.info.max_accel)
            self.info.target_decel_abs = numpy.clip(
                abs(target_accel), 0, self.info.max_decel)

    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out carla.VehicleControl message
        """
        # update current values
        # we calculate the acceleration on ourselves, because we are interested only in
        # the acceleration regarding our driving direction
        # in addition we just filter
        current_time_sec = self.get_current_ros_time().to_sec()
        delta_time = current_time_sec - self.info.current_time_sec
        current_speed = phys.get_vehicle_speed(self.carla_actor)
        if delta_time > 0:
            delta_speed = current_speed - self.info.current_speed
            current_accel = delta_speed / delta_time
            # average filter
            self.info.current_accel = (self.info.current_accel * 4 + current_accel) / 5
        self.info.current_time_sec = current_time_sec
        self.info.current_speed = current_speed
        self.info.current_speed_abs = abs(current_speed)

        # perform actual control
        self.control_steering()
        self.control_stop_and_reverse()
        self.run_pid_control_loop()
        if not self.vehicle_control.hand_brake:
            self.update_drive_vehicle_control_command()

        # send control command out
        self.carla_actor.apply_control(self.vehicle_control)

    def control_steering(self):
        """
        Basic steering control
        """
        self.vehicle_control.steer = self.info.target_steering_angle / \
            self.info.max_steering_angle

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """
        # from this velocity on it is allowed to switch to reverse gear
        standing_still_epsilon = 0.1
        # from this velocity on hand brake is turned on
        full_stop_epsilon = 0.00001

        # auto-control of hand-brake and reverse gear
        self.vehicle_control.hand_brake = False
        if self.info.current_speed_abs < standing_still_epsilon:
            # standing still, change of driving direction allowed
            self.info.status = "standing"
            if self.info.target_speed < 0:
                if not self.vehicle_control.reverse:
                    rospy.loginfo(
                        "VehicleControl: Change of driving direction to reverse")
                    self.vehicle_control.reverse = True
            elif self.info.target_speed > 0:
                if self.vehicle_control.reverse:
                    rospy.loginfo(
                        "VehicleControl: Change of driving direction to forward")
                    self.vehicle_control.reverse = False
            if self.info.target_speed_abs < full_stop_epsilon:
                self.info.status = "full stop"
                self.vehicle_control.hand_brake = True
                self.vehicle_control.brake = 1.0
                self.vehicle_control.throttle = 0.0
                self.set_target_speed(0.)
                self.info.current_speed = 0.
                self.info.current_speed_abs = 0.
                self.info.current_accel = 0.
                self.info.speed_control_accel_target = 0.
                self.info.accel_control_pedal_target = 0.

        elif numpy.sign(self.info.current_speed) * numpy.sign(self.info.target_speed) == -1:
            # requrest for change of driving direction
            # first we have to come to full stop before changing driving
            # direction
            rospy.loginfo("VehicleControl: Request change of driving direction."
                          " v_current={} v_desired={}"
                          " Set desired speed to 0".format(self.info.current_speed,
                                                           self.info.target_speed))
            self.set_target_speed(0.)

    def run_pid_control_loop(self):
        """
        Run the PID control loop for the speed
        """
        self.speed_controller.setpoint = self.info.target_speed_abs
        self.info.speed_control_accel_delta = self.speed_controller(
            self.info.current_speed_abs)
        self.info.speed_control_accel_target = numpy.clip(
            self.info.speed_control_accel_target + self.info.speed_control_accel_delta,
            -self.info.target_decel_abs,
            self.info.target_accel_abs)

        self.accel_controller.setpoint = self.info.speed_control_accel_target
        self.info.accel_control_pedal_delta = self.accel_controller(
            self.info.current_accel)
        self.info.accel_control_pedal_target = numpy.clip(
            self.info.accel_control_pedal_target + self.info.accel_control_pedal_delta,
            -self.info.max_decel,
            self.info.max_accel)

    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """

        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        self.info.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
            self.carla_actor, self.vehicle_control.reverse)
        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        self.info.brake_upper_border = self.info.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.carla_actor)

        if self.info.accel_control_pedal_target > self.info.throttle_lower_border:
            self.info.status = "accelerating"
            self.vehicle_control.brake = 0.0
            # the value has to be normed to max_accel
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            self.vehicle_control.throttle = (
                (self.info.accel_control_pedal_target - self.info.throttle_lower_border) /
                abs(self.info.max_accel))
        elif self.info.accel_control_pedal_target > self.info.brake_upper_border:
            self.info.status = "coasting"
            # no control required
            self.vehicle_control.brake = 0.0
            self.vehicle_control.throttle = 0.0
        else:
            self.info.status = "braking"
            # braking required
            self.vehicle_control.brake = (
                (self.info.brake_upper_border - self.info.accel_control_pedal_target) /
                abs(self.info.max_decel))
            self.vehicle_control.throttle = 0.0

        # finally clip the final control output (should actually never happen)
        self.vehicle_control.brake = numpy.clip(
            self.vehicle_control.brake, 0., 1.)
        self.vehicle_control.throttle = numpy.clip(
            self.vehicle_control.throttle, 0., 1.)
