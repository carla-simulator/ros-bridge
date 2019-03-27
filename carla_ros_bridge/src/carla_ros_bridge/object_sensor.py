#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a object sensor
"""

from derived_object_msgs.msg import ObjectArray
from carla_ros_bridge.vehicle import Vehicle


def get_filtered_objectarray(parent, filtered_id):
    """
    Get a ObjectArray for all available actors, except the one with the filtered_id

    """
    ros_objectlist = ObjectArray()
    ros_objectlist.header = parent.get_msg_header()
    for actor_id, child in parent.child_actors.iteritems():
        # currently only Vehicles are added to the object array
        if filtered_id is not actor_id and isinstance(child, Vehicle):
            ros_objectlist.objects.append(child.get_ros_object_msg())
    return ros_objectlist
