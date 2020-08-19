#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
ROS Vehicle Control usable by scenario-runner
"""

import rospy
import roslaunch
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import carla_common.transforms as trans
from srunner.scenariomanager.actorcontrols.basic_control import BasicControl  # pylint: disable=import-error


class RosVehicleControl(BasicControl):
    """
    ROS Vehicle Control usable by scenario-runner
    """

    def __init__(self, actor, args=None):
        super(RosVehicleControl, self).__init__(actor)
        self._carla_actor = actor
        self._role_name = actor.attributes["role_name"]
        if not self._role_name:
            rospy.logerr("Invalid role_name!")

        rospy.init_node('ros_agent_{}'.format(self._role_name))
        self._current_target_speed = None
        self._current_path = None

        self._target_speed_publisher = rospy.Publisher(
            "/carla/{}/target_speed".format(self._role_name), Float64, queue_size=1, latch=True)

        self._path_publisher = rospy.Publisher(
            "/carla/{}/waypoints".format(self._role_name), Path, queue_size=1, latch=True)

        if "launch" in args and "launch-package" in args:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch_file = args["launch"]
            launch_package = args["launch-package"]

            cli_args = [launch_package, launch_file]
            cli_args.append('role_name:={}'.format(self._role_name))

            # add additional launch parameters
            launch_parameters = []
            for key, value in args.items():
                if not key == "launch" and not key == "launch-package":
                    launch_parameters.append('{}:={}'.format(key, value))
                    cli_args.append('{}:={}'.format(key, value))

            rospy.loginfo("{}: Launching {} from package {} (parameters: {})...".format(
                self._role_name, launch_file, launch_package, launch_parameters))
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
            roslaunch_args = cli_args[2:]
            launch_files = [(roslaunch_file[0], roslaunch_args)]
            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
            parent.start()
            rospy.loginfo("{}: Successfully started ros vehicle control".format(self._role_name))
        else:
            rospy.logerr(
                "{}: Missing value for 'launch' and/or 'launch-package'.".format(self._role_name))

    def update_target_speed(self, speed):
        """
        Update the actor's target speed and set _init_speed to False.

        Args:
            speed (float): New target speed [m/s].
        """
        super(RosVehicleControl, self).update_target_speed(speed)
        rospy.loginfo("{}: Target speed changed to {}".format(self._role_name, speed))
        self._target_speed_publisher.publish(Float64(data=speed))

    def update_waypoints(self, waypoints, start_time=None):
        """
        Execute on tick of the controller's control loop
        """
        super(RosVehicleControl, self).update_waypoints(waypoints, start_time)
        rospy.loginfo("{}: Waypoints changed.".format(self._role_name))
        path = Path()
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = "map"
        for wpt in waypoints:
            path.poses.append(PoseStamped(pose=trans.carla_transform_to_ros_pose(wpt)))
        self._path_publisher.publish(path)

    def reset(self):
        """
        Reset the controller
        """
        if self._carla_actor and self._carla_actor.is_alive:
            self._carla_actor = None
        if self._target_speed_publisher:
            self._target_speed_publisher.unregister()
            self._target_speed_publisher = None
        if self._path_publisher:
            self._path_publisher.unregister()
            self._path_publisher = None

    def run_step(self):
        """
        Execute on tick of the controller's control loop
        """
        pass
