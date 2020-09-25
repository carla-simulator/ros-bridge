# pylint: disable=import-error
import os

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION == 1:
    import rospy
    import tf.transformations as trans

    latch_on = True

    def ros_timestamp(sec=0, nsec=0, from_sec=False):
        if from_sec:
            return rospy.Time.from_sec(sec)
        return rospy.Time(int(sec), int(nsec))

    def ros_ok():
        return not rospy.is_shutdown()

    def ros_shutdown():
        pass

    def destroy_subscription(subscription):
        subscription.unregister()

    def euler_matrix(roll, pitch, yaw):
        return trans.euler_matrix(roll, pitch, yaw)

    def euler_from_quaternion(quaternion):
        return trans.euler_from_quaternion(quaternion)

    def quaternion_from_euler(roll, pitch, yaw):
        return trans.quaternion_from_euler(roll, pitch, yaw)

    def quaternion_from_matrix(matrix):
        return trans.quaternion_from_matrix(matrix)

    def quaternion_multiply(q1, q2):
        return trans.quaternion_multiply(q1, q2)

    class ROSException(rospy.ROSException):
        pass

    class ROSInterruptException(rospy.ROSInterruptException):
        pass

    class QoSProfile():
        def __init__(self, depth=10, durability=None, **kwargs):
            self.depth = depth
            self.latch = bool(durability)

    class CompatibleNode(object):
        def __init__(self, node_name, queue_size=10, latch=False, rospy_init=True, **kwargs):
            if rospy_init:
                rospy.init_node(node_name, anonymous=True)
            self.qos_profile = QoSProfile(depth=queue_size, durability=latch)
            self.callback_group = None

        def destroy(self):
            pass

        def get_param(self, name, alternative_value=None, alternative_name=None):
            if name.startswith('/'):
                raise RuntimeError("Only private parameters are supported.")
            if alternative_value is None:
                return rospy.get_param("~" + name)
            return rospy.get_param("~" + name, alternative_value)

        def logdebug(self, text):
            rospy.logdebug(text)

        def loginfo(self, text):
            rospy.loginfo(text)

        def logwarn(self, text):
            rospy.logwarn(text)

        def logerr(self, text):
            rospy.logerr(text)

        def logfatal(self, arg):
            rospy.logfatal(arg)

        # assymetry in publisher/subscriber method naming due to rclpy having
        # create_publisher method.
        def new_publisher(self, msg_type, topic, qos_profile=None, callback_group=None):
            if qos_profile is None:
                qos_profile = QoSProfile(depth=10, durability=False)
            if callback_group is None:
                callback_group = self.callback_group
            return rospy.Publisher(topic, msg_type, latch=qos_profile.latch,
                                   queue_size=qos_profile.depth)

        def create_subscriber(self, msg_type, topic, callback, qos_profile=None,
                              callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            return rospy.Subscriber(topic, msg_type, callback, queue_size=qos_profile.depth)

        def spin(self, executor=None):
            rospy.spin()

        def shutdown(self):
            rospy.signal_shutdown("")

elif ROS_VERSION == 2:
    from rclpy import Parameter
    from rclpy.exceptions import ROSInterruptException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy
    import rclpy
    from builtin_interfaces.msg import Time

    from transforms3d.euler import euler2mat, euler2quat, quat2euler
    from transforms3d.quaternions import mat2quat, qmult

    latch_on = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL

    def ros_timestamp(sec=0, nsec=0, from_sec=False):
        time = Time()
        if from_sec:
            time.sec = int(sec)
            time.nanosec = int((sec - int(sec)) * 1000000000)
        else:
            time.sec = int(sec)
            time.nanosec = int(nsec)
        return time

    def ros_ok():
        return rclpy.ok()

    def ros_shutdown():
        rclpy.shutdown()

    def destroy_subscription(subscription):
        subscription.destroy()

    def euler_matrix(roll, pitch, yaw):
        return euler2mat(roll, pitch, yaw)

    def euler_from_quaternion(quaternion):
        quat = [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
        return quat2euler(quat)

    def quaternion_from_euler(roll, pitch, yaw):
        quat = euler2quat(roll, pitch, yaw)
        return [quat[1], quat[2], quat[3], quat[0]]

    def quaternion_from_matrix(matrix):
        return mat2quat(matrix)

    def quaternion_multiply(q1, q2):
        q1 = [q1[3], q1[0], q1[1], q1[2]]
        q2 = [q2[3], q2[0], q2[1], q2[2]]
        quat = qmult(q1, q2)
        return [quat[1], quat[2], quat[3], quat[0]]

    class ROSException(Exception):
        pass

    class ROSInterruptException(ROSInterruptException):
        pass

    class CompatibleNode(Node):
        def __init__(self, node_name, queue_size=10, latch=False, rospy_init=True, **kwargs):
            param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
            super(CompatibleNode, self).__init__(node_name, allow_undeclared_parameters=True,
                                                 automatically_declare_parameters_from_overrides=True, parameter_overrides=[param], **kwargs)
            if latch:
                self.qos_profile = QoSProfile(
                    depth=queue_size,
                    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            else:
                self.qos_profile = QoSProfile(depth=queue_size)
            self.callback_group = None

        def destroy(self):
            self.destroy_node()

        def get_param(self, name, alternative_value=None, alternative_name=None):
            if alternative_value is None:
                return self.get_parameter(name).value
            if alternative_name is None:
                alternative_name = name
            return self.get_parameter_or(name,
                                         Parameter(alternative_name, value=alternative_value)
                                         ).value

        def logdebug(self, text):
            self.get_logger().debug(text)

        def loginfo(self, text):
            self.get_logger().info(text)

        def logwarn(self, text):
            self.get_logger().warn(text)

        def logerr(self, text):
            self.get_logger().error(text)

        def logfatal(self, text):
            self.get_logger().fatal(text)

        def new_publisher(self, msg_type, topic, qos_profile=None, callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            if callback_group is None:
                callback_group = self.callback_group
            return self.create_publisher(msg_type, topic, qos_profile,
                                         callback_group=callback_group)

        def create_subscriber(self, msg_type, topic, callback, qos_profile=None,
                              callback_group=None):
            if qos_profile is None:
                qos_profile = self.qos_profile
            if callback_group is None:
                callback_group = self.callback_group
            return self.create_subscription(msg_type, topic, callback, qos_profile,
                                            callback_group=callback_group)

        def spin(self, executor=None):
            rclpy.spin(self, executor)

        def shutdown(self):
            rclpy.shutdown()

else:
    raise NotImplementedError('Make sure you have valid ROS_VERSION env variable.')


def main():
    print('This is a ros1 - ros2 compatibility module.',
          'It is not meant to be executed, but rather imported')


if __name__ == '__main__':
    main()
