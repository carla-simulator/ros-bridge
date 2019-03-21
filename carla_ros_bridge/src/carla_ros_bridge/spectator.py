#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla spectator
"""

from carla_ros_bridge.actor import Actor


class Spectator(Actor):

    """
    Actor implementation details for spectators
    """

    def __init__(self, carla_actor, parent, topic_prefix=None, append_role_name_topic_postfix=True):
        """
        Constructor

        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param topic_prefix: the topic prefix to be used for this actor
        :type topic_prefix: string
        :param append_role_name_topic_postfix: if this flag is set True,
            the role_name of the actor is used as topic postfix
        :type append_role_name_topic_postfix: boolean
        """
        if topic_prefix is None:
            topic_prefix = 'spectator'
        super(Spectator, self).__init__(carla_actor=carla_actor,
                                        parent=parent,
                                        topic_prefix=topic_prefix,
                                        append_role_name_topic_postfix=append_role_name_topic_postfix)  # pylint: disable=line-too-long
