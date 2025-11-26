#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import enum


class DurabilityPolicy(enum.Enum):

    TRANSIENT_LOCAL = 1
    VOLATILE = 2

class ReliabilityPolicy(enum.Enum):

    RELIABLE = 1
    BEST_EFFORT = 2


class QoSProfile(object):
    def __init__(self, depth, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE):
        self.depth = depth
        self.durability = durability
        self.reliability = reliability

    def is_latched(self):
        return self.durability == DurabilityPolicy.TRANSIENT_LOCAL

class QoSProfilePublisher(QoSProfile):
    def __init__(self, depth, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE):
        super(QoSProfilePublisher, self).__init__(depth, durability, reliability)
    
class QoSProfileSubscriber(QoSProfile):
    def __init__(self, depth, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT):
        super(QoSProfileSubscriber, self).__init__(depth, durability, reliability)

