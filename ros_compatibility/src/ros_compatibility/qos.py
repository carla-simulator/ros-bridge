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


class QoSProfile(object):
    def __init__(self, depth, durability=DurabilityPolicy.VOLATILE):
        self.depth = depth
        self.durability = durability

    def is_latched(self):
        return self.durability == DurabilityPolicy.TRANSIENT_LOCAL
