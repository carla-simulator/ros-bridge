#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Control Carla ego vehicle by using AckermannDrive messages
"""
import sys
import datetime
import numpy
import rospy

from simple_pid import PID

from dynamic_reconfigure.server import Server
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleControl  # pylint: disable=no-name-in-module,import-error
from carla_msgs.msg import CarlaEgoVehicleInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_control.msg import EgoVehicleControlInfo  # pylint: disable=no-name-in-module,import-error
from carla_ackermann_control.cfg import EgoVehicleControlParameterConfig  # pylint: disable=no-name-in-module,import-error
import carla_control_physics as phys  # pylint: disable=relative-import


class CarlaAckermannControl(object):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        self.control_loop_rate = rospy.Rate(10)  # 10Hz
        self.lastAckermannMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        # control info
        self.info = EgoVehicleControlInfo()

        # set initial maximum values
        self.vehicle_info_updated(self.vehicle_info)

        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.

        # current values
        self.info.current.time_sec = rospy.get_rostime().to_sec()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.

        # control values
        self.info.status.status = 'n/a'
        self.info.status.speed_control_activation_count = 0
        self.info.status.speed_control_accel_delta = 0.
        self.info.status.speed_control_accel_target = 0.
        self.info.status.accel_control_pedal_delta = 0.
        self.info.status.accel_control_pedal_target = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.

        # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = True

        # PID controller
        # the controller has to run with the simulation time, not with real-time
        #
        # To prevent "float division by zero" within PID controller initialize it with
        # a previous point in time (the error happens because the time doesn't
        # change between initialization and first call, therefore dt is 0)
        sys.modules['simple_pid.PID']._current_time = (       # pylint: disable=protected-access
            lambda: rospy.get_rostime().to_sec() - 0.1)

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
            lambda: rospy.get_rostime().to_sec())

        self.reconfigure_server = Server(
            EgoVehicleControlParameterConfig,
            namespace="/carla/" + self.role_name + "/ackermann_control",
            callback=(lambda config, level: CarlaAckermannControl.reconfigure_pid_parameters(
                self, config, level)))

        # ackermann drive commands
        self.control_subscriber = rospy.Subscriber(
            "/carla/" + self.role_name + "/ackermann_cmd",
            AckermannDrive, self.ackermann_command_updated)

        # current status of the vehicle
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/" + self.role_name + "/vehicle_status",
            CarlaEgoVehicleStatus, self.vehicle_status_updated)

        # vehicle info
        self.vehicle_info_subscriber = rospy.Subscriber(
            "/carla/" + self.role_name + "/vehicle_info",
            CarlaEgoVehicleInfo, self.vehicle_info_updated)

        # to send command to carla
        self.carla_control_publisher = rospy.Publisher(
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            CarlaEgoVehicleControl, queue_size=1)

        # report controller info
        self.control_info_publisher = rospy.Publisher(
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            EgoVehicleControlInfo, queue_size=1)

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscription on AckermannDrive commands.
        Finish the PID controllers.
        Destroy the reconfiguration server

        :return:
        """
        self.control_subscriber = None
        self.speed_controller = None
        self.accel_controller = None
        # first cleanup the server (otherwise leaves a mess behind ;-)
        self.reconfigure_server.set_service.shutdown()
        self.reconfigure_server = None

    def reconfigure_pid_parameters(self, ego_vehicle_control_parameter, _level):
        """
        Callback for dynamic reconfigure call to set the PID parameters

        :param ego_vehicle_control_parameter:
        :type ego_vehicle_control_parameter: \
            carla_ackermann_control.cfg.EgoVehicleControlParameterConfig

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

    def vehicle_status_updated(self, vehicle_status):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """

        # set target values
        self.vehicle_status = vehicle_status

    def vehicle_info_updated(self, vehicle_info):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # set target values
        self.vehicle_info = vehicle_info

        # calculate restrictions
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.vehicle_info)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(
            self.vehicle_info)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(
            self.vehicle_info)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(
            self.vehicle_info)
        self.info.restrictions.min_accel = rospy.get_param('/carla/ackermann_control/min_accel', 1.)
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        self.lastAckermannMsgReceived = datetime.datetime.now()
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
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        # perform actual control
        self.control_steering()
        self.control_stop_and_reverse()
        self.run_speed_control_loop()
        self.run_accel_control_loop()
        if not self.info.output.hand_brake:
            self.update_drive_vehicle_control_command()

            # only send out the Carla Control Command if AckermannDrive messages are
            # received in the last second (e.g. to allows manually controlling the vehicle)
            if (self.lastAckermannMsgReceived + datetime.timedelta(0, 1)) > \
                    datetime.datetime.now():
                self.carla_control_publisher.publish(self.info.output)

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
            self.info.status.status = "standing"
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
                self.info.status.status = "full stop"
                self.info.status.speed_control_accel_target = 0.
                self.info.status.accel_control_pedal_target = 0.
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
            if self.info.status.speed_control_activation_count < 5:
                self.info.status.speed_control_activation_count += 1
        else:
            if self.info.status.speed_control_activation_count > 0:
                self.info.status.speed_control_activation_count -= 1
        # set the auto_mode of the controller accordingly
        self.speed_controller.auto_mode = self.info.status.speed_control_activation_count >= 5

        if self.speed_controller.auto_mode:
            self.speed_controller.setpoint = self.info.target.speed_abs
            self.info.status.speed_control_accel_delta = self.speed_controller(
                self.info.current.speed_abs)

            # clipping borders
            clipping_lower_border = -target_accel_abs
            clipping_upper_border = target_accel_abs
            # per definition of ackermann drive: if zero, then use max value
            if target_accel_abs < epsilon:
                clipping_lower_border = -self.info.restrictions.max_decel
                clipping_upper_border = self.info.restrictions.max_accel
            self.info.status.speed_control_accel_target = numpy.clip(
                self.info.status.speed_control_accel_target +
                self.info.status.speed_control_accel_delta,
                clipping_lower_border, clipping_upper_border)
        else:
            self.info.status.speed_control_accel_delta = 0.
            self.info.status.speed_control_accel_target = self.info.target.accel

    def run_accel_control_loop(self):
        """
        Run the PID control loop for the acceleration
        """
        # setpoint of the acceleration controller is the output of the speed controller
        self.accel_controller.setpoint = self.info.status.speed_control_accel_target
        self.info.status.accel_control_pedal_delta = self.accel_controller(
            self.info.current.accel)
        # @todo: we might want to scale by making use of the the abs-jerk value
        # If the jerk input is big, then the trajectory input expects already quick changes
        # in the acceleration; to respect this we put an additional proportional factor on top
        self.info.status.accel_control_pedal_target = numpy.clip(
            self.info.status.accel_control_pedal_target +
            self.info.status.accel_control_pedal_delta,
            -self.info.restrictions.max_pedal, self.info.restrictions.max_pedal)

    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """

        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        self.info.status.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
            self.vehicle_info, self.vehicle_status, self.info.output.reverse)

        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        self.info.status.brake_upper_border = self.info.status.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.vehicle_info)

        if self.info.status.accel_control_pedal_target > self.info.status.throttle_lower_border:
            self.info.status.status = "accelerating"
            self.info.output.brake = 0.0
            # the value has to be normed to max_pedal
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            self.info.output.throttle = (
                (self.info.status.accel_control_pedal_target -
                 self.info.status.throttle_lower_border) /
                abs(self.info.restrictions.max_pedal))
        elif self.info.status.accel_control_pedal_target > self.info.status.brake_upper_border:
            self.info.status.status = "coasting"
            # no control required
            self.info.output.brake = 0.0
            self.info.output.throttle = 0.0
        else:
            self.info.status.status = "braking"
            # braking required
            self.info.output.brake = (
                (self.info.status.brake_upper_border -
                 self.info.status.accel_control_pedal_target) /
                abs(self.info.restrictions.max_pedal))
            self.info.output.throttle = 0.0

        # finally clip the final control output (should actually never happen)
        self.info.output.brake = numpy.clip(
            self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(
            self.info.output.throttle, 0., 1.)

    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.output.header = self.info.header
        self.control_info_publisher.publish(self.info)

    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = rospy.get_rostime().to_sec()
        delta_time = current_time_sec - self.info.current.time_sec
        current_speed = self.vehicle_status.velocity
        if delta_time > 0:
            delta_speed = current_speed - self.info.current.speed
            current_accel = delta_speed / delta_time
            # average filter
            self.info.current.accel = (self.info.current.accel * 4 + current_accel) / 5
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def run(self):
        """

        Control loop

        :return:
        """

        while not rospy.is_shutdown():
            self.update_current_values()
            self.vehicle_control_cycle()
            self.send_ego_vehicle_control_info_msg()
            try:
                self.control_loop_rate.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    """

    main function

    :return:
    """
    rospy.init_node('carla_ackermann_control', anonymous=True)
    controller = CarlaAckermannControl()
    try:
        controller.run()
    finally:
        del controller
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
