import os

ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    import rospy


    class QoSProfile():
        def __init__(self, **kwargs):
            self.depth = kwargs['depth']


    class CompatibleNode(object):
        def __init__(self, node_name, queue_size=10, rospy_init=True):
            if rospy_init:
                rospy.init_node(node_name, anonymous=True)
            self.qos_profile = QoSProfile(depth=queue_size)
            self.callback_group = None

        def get_param(self, name, alternative_value=None, alternative_name=None):
            if alternative_value is None:
                return rospy.get_param(name)
            return rospy.get_param(name, alternative_value)

        def loginfo(self, text):
            rospy.loginfo(text)

        def logwarn(self, text):
            rospy.logwarn(text)

        def logfatal(self, arg):
            rospy.logfatal(arg)

        # assymetry in publisher/subscriber method naming due to rclpy having
        # create_publisher method.
        def new_publisher(self, msg_type, topic,
                            qos_profile=None, callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            if callback_group is None:
                callback_group = self.callback_group
            return rospy.Publisher(topic, msg_type, queue_size=qos_profile.depth)

        def create_subscriber(self, msg_type, topic,
                              callback, qos_profile=None,
                              callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            return rospy.Subscriber(topic, msg_type,
                                    callback, queue_size=qos_profile.depth)

        def spin(self):
            rospy.spin()

        def shutdown(self):
            pass

elif ROS_VERSION == 2:
    from rclpy.node import Node
    from rclpy import Parameter
    from rclpy.qos import QoSProfile
    import rclpy


    class CompatibleNode(Node):
        def __init__(self, node_name, queue_size=10, rospy_init=True):
            super().__init__(node_name, allow_undeclared_parameters=True,
                             automatically_declare_parameters_from_overrides=True)
            self.logger = self.get_logger()
            self.qos_profile = QoSProfile(depth=queue_size)
            self.callback_group = None

        def get_param(self, name, alternative_value=None, alternative_name=None):
            if alternative_value is None:
                return self.get_parameter(name).value
            if alternative_name is None:
                alternative_name = name
            return self.get_parameter_or(name,
                                         Parameter(alternative_name, value=alternative_value)).value

        def loginfo(self, text):
            self.logger.info(text)

        def logwarn(self, text):
            self.logger.warn(text)

        def logfatal(self, arg):
            self.logger.fatal(arg)

        def new_publisher(self, msg_type, topic,
                            qos_profile=None, callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            if callback_group is None:
                callback_group = self.callback_group
            return self.create_publisher(msg_type, topic,
                                         qos_profile, callback_group=callback_group)

        def create_subscriber(self, msg_type, topic,
                              callback, qos_profile=None, callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            if callback_group is None:
                callback_group = self.callback_group
            return self.create_subscription(msg_type, topic,
                                            callback, qos_profile,
                                            callback_group=callback_group)

        def spin(self):
            rclpy.spin(self)

        def shutdown(self):
            rclpy.shutdown()

else:
    raise NotImplementedError("Make sure you have valid " +
                              "ROS_VERSION env variable");


def main():
    print('This is a ros1 - ros2 compatibility module.',
          "It's not meant to be executed, but rather imported")


if __name__ == '__main__':
    main()
