#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic objects
"""

from carla_ros_bridge.actor import Actor


class Traffic(Actor):

    """
    Actor implementation details for traffic objects
    """

    def __init__(self, carla_actor, parent, binding, topic_prefix=None):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        """
        if topic_prefix is None:
            topic_prefix = 'traffic'
        super(Traffic, self).__init__(carla_actor=carla_actor,
                                      parent=parent,
                                      topic_prefix=topic_prefix,
                                      binding=binding)


class TrafficLight(Traffic):

    """
    Traffic implementation details for traffic lights
    """

    def __init__(self, carla_actor, parent, binding):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.TrafficLight
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        """
        topic_prefix = 'traffic.traffic_light'
        super(TrafficLight, self).__init__(carla_actor=carla_actor,
                                           parent=parent,
                                           binding=binding,
                                           topic_prefix=topic_prefix)
