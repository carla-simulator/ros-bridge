#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla obstacle sensor
"""

import math

from carla_ros_bridge.sensor import Sensor

from carla_msgs.msg import CarlaObstacle


class ObstacleSensor(Sensor):

    """
    Actor implementation details for obstacle sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor : carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(ObstacleSensor, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        self.obstacle_publisher = node.new_publisher(CarlaObstacle, self.get_topic_prefix(), qos_profile=10)
        self.listen()

    def destroy(self):
        super(ObstacleSensor, self).destroy()
        self.node.destroy_publisher(self.obstacle_publisher)
    
    def get_vector_length_squared(self,carla_vector):
        """
        Calculate the squared length of a carla_vector
        :param carla_vector: the carla vector
        :type carla_vector: carla.Vector3D
        :return: squared vector length
        :rtype: float64
        """
        return carla_vector.x * carla_vector.x + \
            carla_vector.y * carla_vector.y + \
            carla_vector.z * carla_vector.z

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_obstacle_measurement):
        """
        Function to transform a received obstacle measurement into a ROS Obstacle message

        :param carla_obstacle_measurement: carla obstacle measurement object
        """
        obstacle_msg = CarlaObstacle()
        obstacle_msg.header = self.get_msg_header(timestamp=carla_obstacle_measurement.timestamp)
        obstacle_msg.velocity = math.sqrt(self.get_vector_length_squared(carla_obstacle_measurement.other_actor.get_velocity()))
        obstacle_msg.distance = carla_obstacle_measurement.distance


        self.obstacle_publisher.publish(obstacle_msg)
