#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import os


def get_ros_version():

    ros_version = int(os.environ.get("ROS_VERSION", 0))

    if ros_version not in (1, 2):
        raise RuntimeError("Make sure you have a valid `ROS_VERSION` env variable set.")

    return ros_version
