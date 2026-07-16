#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

from std_msgs.msg import Bool  # pylint: disable=import-error


"""
Classes to handle Carla vehicles
"""

from carla_ros_client.actor import Actor

class Vehicle(Actor):

    """
    Actor implementation details for vehicles
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_client.Parent
        :param node: node-handle
        :type node: carla_ros_client.CarlaRosClient
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        """

        super(Vehicle, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=node,
                                      carla_actor=carla_actor)
        
        self.enable_autopilot_subscriber = node.new_subscription(
            Bool,
            self.get_topic_prefix() + "/enable_autopilot",
            self.enable_autopilot_updated,
            qos_profile=10)
        
    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        self.node.destroy_subscription(self.enable_autopilot_subscriber)


    def enable_autopilot_updated(self, enable_auto_pilot):
        """
        Enable/disable auto pilot

        :param enable_auto_pilot: should the autopilot be enabled?
        :type enable_auto_pilot: std_msgs.Bool
        :return:
        """
        if enable_auto_pilot.data:
            if self.carla_actor.get_world().get_settings().synchronous_mode:
                # ensure traffic manager is in synchronous mode as well, otherwise the autopilot will not work correctly
                self.node.carla_client.get_trafficmanager().set_synchronous_mode(True)
            self.node.loginfo("vehicle[{}]: Autopilot enabled".format(self.uid))
        else:
            self.node.loginfo("vehicle[{}]: Autopilot disabled".format(self.uid))
        
        self.carla_actor.set_autopilot(enable_auto_pilot.data)

