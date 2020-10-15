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

    class ServiceException(rospy.ServiceException):
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

        def new_rate(self, rate):
            return rospy.Rate(rate)

        def new_timer(self, timer_period_sec, callback):
            return rospy.Timer(rospy.Duration(timer_period_sec), callback)

        def wait_for_one_message(self, topic, topic_type, timeout=None, qos_profile=None):
            return rospy.wait_for_message(topic, topic_type, timeout)

        def new_service(self, srv_type, srv_name, callback, qos_profile=None, callback_group=None):
            return rospy.Service(srv_name, srv_type, callback)

        def create_service_client(self, service_name, service, callback_group=None):
            rospy.wait_for_service(service_name)
            try:
                client = rospy.ServiceProxy(service_name, service)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            return client

        def call_service(self, client, req):
            try:
                return client(req)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        def spin(self, executor=None):
            rospy.spin()

        def get_time(self):
            return rospy.get_time()

        def shutdown(self):
            rospy.signal_shutdown("")

        def on_shutdown(self, handler):
            rospy.on_shutdown(handler)

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

    class WaitForMessageHelper(object):
        def __init__(self):
            self.msg = None

        def callback(self, msg):
            if self.msg is None:
                self.msg = msg

    class ROSException(Exception):
        pass

    class ROSInterruptException(ROSInterruptException):
        pass

    class ServiceException(Exception):
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

        def new_rate(self, rate):
            return self.create_rate(rate)

        def new_timer(self, timer_period_sec, callback):
            return self.create_timer(timer_period_sec, callback)

        def wait_for_one_message(self, topic, topic_type, timeout=None, qos_profile=None):
            s = None
            spin_timeout = 0.5
            loop_reps = -1
            if timeout is not None:
                loop_reps = timeout // spin_timeout + 1
            wfm = WaitForMessageHelper()
            try:
                s = self.create_subscriber(topic_type, topic, wfm.callback, qos_profile=qos_profile)
                while ros_ok() and wfm.msg is None:
                    rclpy.spin_once(self, timeout_sec=spin_timeout)
                    loop_reps = loop_reps - 1
                    if loop_reps == 0:
                        raise ROSException
            finally:
                if s is not None:
                    self.destroy_subscription(s)
            return wfm.msg

        def new_service(self, srv_type, srv_name, callback, qos_profile=None, callback_group=None):
            return self.create_service(srv_type, srv_name, callback)

        def create_service_client(self, service_name, service, callback_group=None):
            client = self.create_client(service, service_name, callback_group=callback_group)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            return client

        def call_service(self, client, req):
            resp = client.call_async(req)
            rclpy.spin_until_future_complete(self, resp)
            try:
                result = resp.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                return result

        def spin(self, executor=None):
            rclpy.spin(self, executor)

        def get_time(self):
            t = self.get_clock().now()
            t = t.seconds_nanoseconds()
            return float(t[0] + (t[1] / 10**9))

        def shutdown(self):
            rclpy.shutdown()

        def on_shutdown(self, handler):
            rclpy.get_default_context().on_shutdown(handler)


else:
    raise NotImplementedError('Make sure you have valid ROS_VERSION env variable.')


def main():
    print('This is a ros1 - ros2 compatibility module.',
          'It is not meant to be executed, but rather imported')


if __name__ == '__main__':
    main()
