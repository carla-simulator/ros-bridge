#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Labs.
#
# authors: Bernd Gassmann (bernd.gassmann@intel.com)
#
"""
Classes to handle Carla vehicles
"""
import sys
import datetime
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
        # control info
        self.info = EgoVehicleControlInfo()

        # maximum values
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.carla_actor)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(
            self.carla_actor)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(
            self.carla_actor)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(
            self.carla_actor)
        self.info.restrictions.min_accel = 0.
        self.info.restrictions.max_pedal = 1.0

        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.

        # current values
        self.info.current.time_sec = self.get_current_ros_time().to_sec()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.

        # control values
        self.info.state.status = 'n/a'
        self.info.state.speed_control_activation_count = 0
        self.info.state.speed_control_accel_delta = 0.
        self.info.state.speed_control_accel_target = 0.
        self.info.state.accel_control_pedal_delta = 0.
        self.info.state.accel_control_pedal_target = 0.
        self.info.state.brake_upper_border = 0.
        self.info.state.throttle_lower_border = 0.

        # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = True

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

    def apply_control(self):
        """
        Apply current control output to CARLA

        :return:
        """
        vehicle_control = VehicleControl()
        vehicle_control.hand_brake = self.info.output.hand_brake
        vehicle_control.brake = self.info.output.brake
        vehicle_control.steer = self.info.output.steer
        vehicle_control.throttle = self.info.output.throttle
        vehicle_control.reverse = self.info.output.reverse

        self.carla_actor.apply_control(vehicle_control)

    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = self.get_current_ros_time().to_sec()
        delta_time = current_time_sec - self.info.current.time_sec
        current_speed = phys.get_vehicle_speed(self.carla_actor)
        if delta_time > 0:
            delta_speed = current_speed - self.info.current.speed
            current_accel = delta_speed / delta_time
            # average filter
            self.info.current.accel = (self.info.current.accel * 4 + current_accel) / 5
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def vehicle_control_cycle(self):
        """
        Function to perform a control cycle.

        To be overridden by derived control classes

        :return:
        """
        pass

    def update(self):
        """
        Function (override) to update this object.

        On update ego vehicle calculates and sends the new values for VehicleControl()

        :return:
        """
        super(EgoVehicle, self).update()
        self.update_current_values()
        self.vehicle_control_cycle()
        self.send_ego_vehicle_control_info_msg()

    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ros_bridge.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.header = self.get_msg_header()
        self.info.output.header = self.info.header
        self.publish_ros_message(
            self.topic_name() + "/ego_vehicle_control_info", self.info)


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

        :param ros_vehicle_control: current vehicle control input received via ROS
        :type self.info.output: carla_ros_bridge.msg.CarlaVehicleControl
        :return:
        """
        self.info.output = ros_vehicle_control
        self.apply_control()


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

        # adapt initial values
        self.info.restrictions.min_accel = parent.get_param('ego_vehicle').get('min_accel', 1.)
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        #
        # To prevent "float division by zero" within PID controller initialize it with
        # a previous point in time (the error happens because the time doesn't
        # change between initialization and first call, therefore dt is 0)
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: AckermannControlVehicle.get_current_ros_time(self).to_sec() - 0.1)

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

        # use the correct time for further calculations
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: AckermannControlVehicle.get_current_ros_time(self).to_sec())

        self.reconfigure_server = Server(
            EgoVehicleControlParameterConfig,
            namespace=self.topic_name(),
            callback=(lambda config, level: AckermannControlVehicle.reconfigure_pid_parameters(
                self, config, level)))

        # ROS subscriber
        # the lastMsgReceived is updated within the callback
        # (it's used to stop updating the carla client control if no new messages were received)
        self.lastMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
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

    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Convert a Ackerman drive msg into carla control msg

        This function gets called whenever a ROS message is received via
        '/carla/ego_vehicle/vehicle_control_cmd' topic.
        The received ROS message is converted into carla.VehicleControl command and
        sent to CARLA.
        This brigde is not responsible for any restrictions on velocity or steering.
        It's just forwarding the ROS input to CARLA

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # update the last reception timestamp
        self.lastMsgReceived = datetime.datetime.now()

        # set target values
        self.set_target_steering_angle(ros_ackermann_drive.steering_angle)
        self.set_target_speed(ros_ackermann_drive.speed)
        self.set_target_accel(ros_ackermann_drive.acceleration)
        self.set_target_jerk(ros_ackermann_drive.jerk)

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        self.info.target.steering_angle = -target_steering_angle
        if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
            rospy.logerr("Max steering angle reached, clipping value")
            self.info.target.steering_angle = numpy.clip(
                self.info.target.steering_angle,
                -self.info.restrictions.max_steering_angle,
                self.info.restrictions.max_steering_angle)

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            rospy.logerr("Max speed reached, clipping value")
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        epsilon = 0.00001
        # if speed is set to zero, then use max decel value
        if self.info.target.speed_abs < epsilon:
            self.info.target.accel = -self.info.restrictions.max_decel
        else:
            self.info.target.accel = numpy.clip(
                target_accel, -self.info.restrictions.max_decel, self.info.restrictions.max_accel)

    def set_target_jerk(self, target_jerk):
        """
        set target accel
        """
        self.info.target.jerk = target_jerk

    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out carla.VehicleControl message
        """
        # perform actual control
        self.control_steering()
        self.control_stop_and_reverse()
        self.run_speed_control_loop()
        self.run_accel_control_loop()
        if not self.info.output.hand_brake:
            self.update_drive_vehicle_control_command()

        # apply control command to CARLA
        # (only if a control message was received in the last 3 seconds. This is a workaround
        # for rospy.subscriber.get_num_connections() not working in Ubuntu 16.04)
        if (self.lastMsgReceived + datetime.timedelta(0, 3)) > datetime.datetime.now():
            self.apply_control()

    def control_steering(self):
        """
        Basic steering control
        """
        self.info.output.steer = self.info.target.steering_angle / \
            self.info.restrictions.max_steering_angle

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """
        # from this velocity on it is allowed to switch to reverse gear
        standing_still_epsilon = 0.1
        # from this velocity on hand brake is turned on
        full_stop_epsilon = 0.00001

        # auto-control of hand-brake and reverse gear
        self.info.output.hand_brake = False
        if self.info.current.speed_abs < standing_still_epsilon:
            # standing still, change of driving direction allowed
            self.info.state.status = "standing"
            if self.info.target.speed < 0:
                if not self.info.output.reverse:
                    rospy.loginfo(
                        "VehicleControl: Change of driving direction to reverse")
                    self.info.output.reverse = True
            elif self.info.target.speed > 0:
                if self.info.output.reverse:
                    rospy.loginfo(
                        "VehicleControl: Change of driving direction to forward")
                    self.info.output.reverse = False
            if self.info.target.speed_abs < full_stop_epsilon:
                self.info.state.status = "full stop"
                self.info.state.speed_control_accel_target = 0.
                self.info.state.accel_control_pedal_target = 0.
                self.set_target_speed(0.)
                self.info.current.speed = 0.
                self.info.current.speed_abs = 0.
                self.info.current.accel = 0.
                self.info.output.hand_brake = True
                self.info.output.brake = 1.0
                self.info.output.throttle = 0.0

        elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
            # requrest for change of driving direction
            # first we have to come to full stop before changing driving
            # direction
            rospy.loginfo("VehicleControl: Request change of driving direction."
                          " v_current={} v_desired={}"
                          " Set desired speed to 0".format(self.info.current.speed,
                                                           self.info.target.speed))
            self.set_target_speed(0.)

    def run_speed_control_loop(self):
        """
        Run the PID control loop for the speed

        The speed control is only activated if desired acceleration is moderate
        otherwhise we try to follow the desired acceleration values

        Reasoning behind:

        An autonomous vehicle calculates a trajectory including position and velocities.
        The ackermann drive is derived directly from that trajectory.
        The acceleration and jerk values provided by the ackermann drive command
        reflect already the speed profile of the trajectory.
        It makes no sense to try to mimick this a-priori knowledge by the speed PID
        controller.
        =>
        The speed controller is mainly responsible to keep the speed.
        On expected speed changes, the speed control loop is disabled
        """
        epsilon = 0.00001
        target_accel_abs = abs(self.info.target.accel)
        if target_accel_abs < self.info.restrictions.min_accel:
            if self.info.state.speed_control_activation_count < 5:
                self.info.state.speed_control_activation_count += 1
        else:
            if self.info.state.speed_control_activation_count > 0:
                self.info.state.speed_control_activation_count -= 1
        # set the auto_mode of the controller accordingly
        self.speed_controller.auto_mode = self.info.state.speed_control_activation_count >= 5

        if self.speed_controller.auto_mode:
            self.speed_controller.setpoint = self.info.target.speed_abs
            self.info.state.speed_control_accel_delta = self.speed_controller(
                self.info.current.speed_abs)

            # clipping borders
            clipping_lower_border = -target_accel_abs
            clipping_upper_border = target_accel_abs
            # per definition of ackermann drive: if zero, then use max value
            if target_accel_abs < epsilon:
                clipping_lower_border = -self.info.restrictions.max_decel
                clipping_upper_border = self.info.restrictions.max_accel
            self.info.state.speed_control_accel_target = numpy.clip(
                self.info.state.speed_control_accel_target +
                self.info.state.speed_control_accel_delta,
                clipping_lower_border, clipping_upper_border)
        else:
            self.info.state.speed_control_accel_delta = 0.
            self.info.state.speed_control_accel_target = self.info.target.accel

    def run_accel_control_loop(self):
        """
        Run the PID control loop for the acceleration
        """
        # setpoint of the acceleration controller is the output of the speed controller
        self.accel_controller.setpoint = self.info.state.speed_control_accel_target
        self.info.state.accel_control_pedal_delta = self.accel_controller(
            self.info.current.accel)
        # @todo: we might want to scale by making use of the the abs-jerk value
        # If the jerk input is big, then the trajectory input expects already quick changes
        # in the acceleration; to respect this we put an additional proportional factor on top
        self.info.state.accel_control_pedal_target = numpy.clip(
            self.info.state.accel_control_pedal_target + self.info.state.accel_control_pedal_delta,
            -self.info.restrictions.max_pedal, self.info.restrictions.max_pedal)

    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """

        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        self.info.state.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
            self.carla_actor, self.info.output.reverse)
        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        self.info.state.brake_upper_border = self.info.state.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.carla_actor)

        if self.info.state.accel_control_pedal_target > self.info.state.throttle_lower_border:
            self.info.state.status = "accelerating"
            self.info.output.brake = 0.0
            # the value has to be normed to max_pedal
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            self.info.output.throttle = (
                (self.info.state.accel_control_pedal_target -
                 self.info.state.throttle_lower_border) /
                abs(self.info.restrictions.max_pedal))
        elif self.info.state.accel_control_pedal_target > self.info.state.brake_upper_border:
            self.info.state.status = "coasting"
            # no control required
            self.info.output.brake = 0.0
            self.info.output.throttle = 0.0
        else:
            self.info.state.status = "braking"
            # braking required
            self.info.output.brake = (
                (self.info.state.brake_upper_border -
                 self.info.state.accel_control_pedal_target) /
                abs(self.info.restrictions.max_pedal))
            self.info.output.throttle = 0.0

        # finally clip the final control output (should actually never happen)
        self.info.output.brake = numpy.clip(
            self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(
            self.info.output.throttle, 0., 1.)
