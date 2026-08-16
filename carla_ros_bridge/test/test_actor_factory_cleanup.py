#!/usr/bin/env python

#
# Copyright (c) 2026 CARLA Simulator Team
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

"""Unit tests for best-effort actor cleanup."""

# The queue and active-actor set are the behavior boundary under test, and the
# lightweight test doubles intentionally expose only one production method.
# pylint: disable=protected-access,too-few-public-methods,attribute-defined-outside-init

import queue
import unittest
from threading import Lock

from carla_ros_bridge.actor_factory import ActorFactory


class RecordingNode:
    """Record cleanup warnings without requiring a ROS node."""

    def __init__(self):
        """Initialize the warning collector."""
        self.warnings = []

    def logwarn(self, message):
        """Collect one warning emitted by ActorFactory."""
        self.warnings.append(message)


class EmptyWorld:
    """Represent a CARLA world with no newly discovered actors."""

    @staticmethod
    def get_actors():
        """Return the empty actor collection used during queue draining."""
        return []


class FailingActorFactory(ActorFactory):
    """Fail the first external destroy operation and record every attempt."""

    def _destroy_object(self, actor_id, delete_actor):
        """Record the actor ID and raise only for the first test actor."""
        self.attempted_actor_ids.append(actor_id)
        if actor_id == 1:
            raise RuntimeError("first actor failed")


class TestActorFactoryCleanup(unittest.TestCase):
    """Verify deferred destroy tasks use best-effort cleanup semantics."""

    def test_destroy_task_failure_does_not_abort_remaining_cleanup(self):
        """A failed queued actor must not prevent the next actor from being attempted."""
        factory = FailingActorFactory.__new__(FailingActorFactory)
        factory.node = RecordingNode()
        factory.world = EmptyWorld()
        factory._active_actors = set()
        factory.actors = {}
        factory.lock = Lock()
        factory.spawn_lock = Lock()
        factory._task_queue = queue.Queue()
        factory.attempted_actor_ids = []
        factory._task_queue.put((factory.TaskType.DESTROY_ACTOR, (1, None)))
        factory._task_queue.put((factory.TaskType.DESTROY_ACTOR, (2, None)))

        factory.update_available_objects()

        self.assertEqual([1, 2], factory.attempted_actor_ids)
        self.assertTrue(factory._task_queue.empty())
        self.assertEqual(1, len(factory.node.warnings))
        self.assertIn("1", factory.node.warnings[0])


if __name__ == "__main__":
    unittest.main()
