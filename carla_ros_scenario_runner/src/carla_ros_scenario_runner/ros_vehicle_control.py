#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control usable by scenario-runner
"""

import os

from srunner.scenariomanager.actorcontrols.basic_control import BasicControl  # pylint: disable=import-error

import carla_common.transforms as trans
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_scenario_runner.application_runner import ApplicationRunner

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64

ROS_VERSION = roscomp.get_ros_version()


class RosVehicleControl(BasicControl):

    def __init__(self, actor, args=None):
        super(RosVehicleControl, self).__init__(actor)

        self._carla_actor = actor
        self._role_name = actor.attributes["role_name"]
        if not self._role_name:
            roscomp.logerr("Invalid role_name")

        self._path_topic_name = "waypoints"
        if "path_topic_name" in args:
            self._path_topic_name = args["path_topic_name"]

        roscomp.init("ros_agent_{}".format(self._role_name), args=None)

        self.node = CompatibleNode('ros_agent_{}'.format(self._role_name))

        self._current_target_speed = None
        self._current_path = None
        self.controller_launch = None

        self._target_speed_publisher = self.node.new_publisher(
            Float64,
            "/carla/{}/target_speed".format(self._role_name),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.loginfo("Publishing target_speed on /carla/{}/target_speed".format(self._role_name))

        self._path_publisher = self.node.new_publisher(
            Path,
            "/carla/{}/{}".format(self._role_name, self._path_topic_name),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.loginfo("Publishing path on /carla/{}/{}".format(self._role_name, self._path_topic_name))

        if "launch" in args and "launch-package" in args:

            launch_file = args["launch"]
            launch_package = args["launch-package"]

            if ROS_VERSION == 1:
                executable = "roslaunch"
                cli_args = [launch_package, launch_file]
            elif ROS_VERSION == 2:
                executable = "ros2 launch"
                cli_args = [launch_package, launch_file + '.py']
            cli_args.append('role_name:={}'.format(self._role_name))

            # add additional launch parameters
            launch_parameters = []
            for key, value in args.items():
                if not key == "launch" and not key == "launch-package" and not key == "path_topic_name":
                    launch_parameters.append('{}:={}'.format(key, value))
                    cli_args.append('{}:={}'.format(key, value))

            self.node.loginfo("{}: Launching {} from package {} (parameters: {})...".format(
                self._role_name, launch_file, launch_package, launch_parameters))

            cmdline = [executable] + cli_args
            self.controller_launch = ApplicationRunner(
                self.controller_runner_status_updated, self.controller_runner_log, 'RosVehicleControl: launching controller node')
            self.controller_launch.execute(cmdline, env=os.environ,)

            self.node.loginfo(
                "{}: Successfully started ros vehicle control".format(self._role_name))
        else:
            self.node.logwarn(
                "{}: Missing value for 'launch' and/or 'launch-package'.".format(self._role_name))

    def controller_runner_log(self, log):  # pylint: disable=no-self-use
        """
        Callback for application logs
        """
        self.node.logwarn("[Controller]{}".format(log))

    def controller_runner_status_updated(self, status):
        """
        Executed from application runner whenever the status changed
        """
        self.node.loginfo("Controller status is: {}".format(status))

    def update_target_speed(self, speed):
        super(RosVehicleControl, self).update_target_speed(speed)
        self.node.loginfo("{}: Target speed changed to {}".format(self._role_name, speed))
        self._target_speed_publisher.publish(Float64(data=speed))

    def update_waypoints(self, waypoints, start_time=None):
        super(RosVehicleControl, self).update_waypoints(waypoints, start_time)
        self.node.loginfo("{}: Waypoints changed.".format(self._role_name))
        path = Path()
        path.header.stamp = roscomp.ros_timestamp(sec=self.node.get_time(), from_sec=True)
        path.header.frame_id = "map"
        for wpt in waypoints:
            print(wpt)
            path.poses.append(PoseStamped(pose=trans.carla_transform_to_ros_pose(wpt)))
        self._path_publisher.publish(path)

    def reset(self):
        # set target speed to zero before closing as the controller can take time to shutdown
        self.update_target_speed(0.)
        if self.controller_launch and self.controller_launch.is_running():
            self.controller_launch.shutdown()
        if self._carla_actor and self._carla_actor.is_alive:
            self._carla_actor = None
        if self._target_speed_publisher:
            self.node.destroy_subscription(self._target_speed_publisher)
            self._target_speed_publisher = None
        if self._path_publisher:
            self.node.destroy_subscription(self._path_publisher)
            self._path_publisher = None

    def run_step(self):
        pass
