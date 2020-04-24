#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
A basic AD agent using CARLA waypoints
"""
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from carla_msgs.msg import CarlaWalkerControl
from basic_agent import BasicAgent


class CarlaAdAgent(object):
    """
    A basic AD agent using CARLA waypoints
    """

    def __init__(self, role_name, target_speed, avoid_risk,):
        """
        Constructor
        """
        self._route_assigned = False
        self._global_plan = None
        self._agent = None
        self._target_speed = target_speed
        rospy.on_shutdown(self.on_shutdown)

        actor_id = None
        self.control_publisher = rospy.Publisher(
            "/carla/{}/walker_control_cmd".format(role_name), CarlaWalkerControl, queue_size=1)

        self._route_subscriber = rospy.Subscriber(
            "/carla/{}/waypoints".format(role_name), Path, self.path_updated)

        self._target_speed_subscriber = rospy.Subscriber(
            "/carla/{}/target_speed".format(role_name), Float64, self.target_speed_updated)

        self._agent = BasicAgent(role_name, actor_id,  # pylint: disable=no-member
                                 avoid_risk)

    def on_shutdown(self):
        """
        callback on shutdown
        """
        rospy.loginfo("Shutting down, stopping ego vehicle...")
        if self._agent:
            self.control_publisher.publish(self._agent.emergency_stop())

    def target_speed_updated(self, target_speed):
        """
        callback on new target speed
        """
        rospy.loginfo("New target speed received: {}".format(target_speed.data))
        self._target_speed = target_speed.data

    def path_updated(self, path):
        """
        callback on new route
        """
        rospy.loginfo("New plan with {} waypoints received.".format(len(path.poses)))
        if self._agent:
            self.control_publisher.publish(self._agent.emergency_stop())
        self._global_plan = path
        self._route_assigned = False

    def run_step(self):
        """
        Execute one step of navigation.
        """
        control = CarlaWalkerControl()

        if not self._agent:
            rospy.loginfo("Waiting for ego vehicle...")
            return control

        if not self._route_assigned and self._global_plan:
            rospy.loginfo("Assigning plan...")
            self._agent.set_global_plan(  # pylint: disable=protected-access
                self._global_plan.poses)
            self._route_assigned = True
        else:
            control,finished = self._agent.run_step(self._target_speed)
            if finished:
                self._global_plan = None
                self._route_assigned = False

        return control

    def run(self):
        """

        Control loop

        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._global_plan:
                control = self.run_step()
                if control:
                    self.control_publisher.publish(control)
            else:
                try:
                    r.sleep()
                except rospy.ROSInterruptException:
                    pass



def main():
    """

    main function

    :return:
    """
    rospy.init_node('carla_ad_agent', anonymous=True)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    target_speed = rospy.get_param("~target_speed", 20)
    avoid_risk = rospy.get_param("~avoid_risk", True)
    controller = CarlaAdAgent(role_name, target_speed, avoid_risk)
    try:
        controller.run()
    finally:
        del controller
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
