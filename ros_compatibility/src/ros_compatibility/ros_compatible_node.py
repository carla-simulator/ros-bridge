# pylint: disable=import-error
import os

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

if ROS_VERSION not in (1, 2):
    raise NotImplementedError("Make sure you have a valid ROS_VERSION env variable set.")

if ROS_VERSION == 1:
    import rospy
elif ROS_VERSION == 2:
    import rclpy
else:
    raise NotImplementedError('Make sure you have valid ROS_VERSION env variable.')

if ROS_VERSION == 1:

    latch_on = True

    def ros_init(args=None):
        pass

    def ros_timestamp(sec=0, nsec=0, from_sec=False):
        if from_sec:
            return rospy.Time.from_sec(sec)
        return rospy.Time(int(sec), int(nsec))

    def ros_ok():
        return not rospy.is_shutdown()

    def ros_shutdown():
        pass

    def ros_on_shutdown(handler):
        rospy.on_shutdown(handler)

    def logdebug(log):
        rospy.logdebug(log)

    def loginfo(log):
        rospy.loginfo(log)

    def logwarn(log):
        rospy.logwarn(log)

    def logerr(log):
        rospy.logerr(log)

    def logfatal(log):
        rospy.logfatal(log)

    def get_service_request(service_type):
        ros1_classname = service_type.__name__ + "Request"
        module = ".".join(service_type.__module__.split(".")[:-1])
        request_class = __import__(module, fromlist=[ros1_classname])
        return getattr(request_class, ros1_classname)()

    def get_service_response(service_type):
        ros1_classname = service_type.__name__ + "Response"
        module = ".".join(service_type.__module__.split(".")[:-1])
        request_class = __import__(module, fromlist=[ros1_classname])
        return getattr(request_class, ros1_classname)()

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

        def destroy_service(self, service):
            service.shutdown()

        def destroy_subscription(self, subscription):
            subscription.unregister()

        def destroy_publisher(self, publisher):
            publisher.unregister()

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

        def wait_for_one_message(self, topic, topic_type, timeout=None, qos_profile=None, executor=None):
            try:
                return rospy.wait_for_message(topic, topic_type, timeout)
            except rospy.ROSException as e:
                raise ROSException(e)

        def new_service(self, srv_type, srv_name, callback, qos_profile=None, callback_group=None):
            return rospy.Service(srv_name, srv_type, callback)

        def create_service_client(self, service_name, service, timeout_sec=None, callback_group=None):
            if timeout_sec is not None:
                timeout = timeout_sec * 1000
            else:
                timeout = timeout_sec
            try:
                rospy.wait_for_service(service_name, timeout=timeout)
                client = rospy.ServiceProxy(service_name, service)
            except rospy.ServiceException as e:
                raise ServiceException(e)
            except rospy.ROSException as e:
                raise ROSException(e)
            return client

        def call_service(self, client, req, timeout_ros2=None, executor=None):
            try:
                return client(req)
            except rospy.ServiceException as e:
                raise ServiceException(e)

        def spin(self, executor=None):
            rospy.spin()

        def get_time(self):
            return rospy.get_time()

        def shutdown(self):
            rospy.signal_shutdown("")

        def on_shutdown(self, handler):
            rospy.on_shutdown(handler)

elif ROS_VERSION == 2:
    import time
    from rclpy import Parameter
    from rclpy.exceptions import ROSInterruptException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy
    from builtin_interfaces.msg import Time

    latch_on = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
    def dummy_handler(): pass
    shutdown_handler = dummy_handler

    def ros_init(args=None):
        rclpy.init(args=args)

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
        shutdown_handler()
        rclpy.shutdown()

    def ros_on_shutdown(handler):
        global shutdown_handler
        shutdown_handler = handler

    def logdebug(log):
        rclpy.logging.get_logger("default").debug(log)

    def loginfo(log):
        rclpy.logging.get_logger("default").info(log)

    def logwarn(log):
        rclpy.logging.get_logger("default").warn(log)

    def logerr(log):
        rclpy.logging.get_logger("default").error(log)

    def logfatal(log):
        rclpy.logging.get_logger("default").fatal(log)

    class WaitForMessageHelper(object):
        def __init__(self):
            self.msg = None

        def callback(self, msg):
            if self.msg is None:
                self.msg = msg

    def get_service_request(service_type):
        return service_type.Request()

    def get_service_response(service_type):
        return service_type.Response()

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

        def wait_for_one_message(self, topic, topic_type, timeout=None, qos_profile=None, executor=None):
            s = None
            wfm = WaitForMessageHelper()
            try:
                s = self.create_subscriber(topic_type, topic, wfm.callback, qos_profile=qos_profile)
                if timeout is not None:
                    timeout_t = time.time() + timeout
                    while ros_ok() and wfm.msg is None:
                        time.sleep(0.01)
                        if executor is None:
                            rclpy.spin_once(self, timeout_sec=0)
                        else:
                            executor.spin_once(timeout_sec=0)
                        if time.time() >= timeout_t:
                            raise ROSException
                else:
                    while ros_ok() and wfm.msg is None:
                        time.sleep(0.01)
                        if executor is None:
                            rclpy.spin_once(self, timeout_sec=0)
                        else:
                            executor.spin_once(timeout_sec=0)
            finally:
                if s is not None:
                    self.destroy_subscription(s)
            return wfm.msg

        def new_service(self, srv_type, srv_name, callback, qos_profile=None, callback_group=None):
            return self.create_service(srv_type, srv_name, callback, callback_group=callback_group)

        def create_service_client(self, service_name, service, timeout_sec=None, callback_group=None):
            client = self.create_client(service, service_name, callback_group=callback_group)
            status = client.wait_for_service(timeout_sec=timeout_sec)
            if status is True:
                return client
            else:
                raise ROSException("Timeout of {}sec while waiting for service".format(timeout_sec))

        def call_service(self, client, req, timeout_ros2=None, executor=None):
            # uses the asynchronous call function but behaves like the synchronous call
            # this is done because the basic synchronous call function doesn't raise
            # an error when trying to call a service that is not alive anymore
            future = client.call_async(req)
            if executor is None:
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_ros2)
            else:
                executor.spin_until_future_complete(future, timeout_sec=timeout_ros2)
            if future.done():
                return future.result()
            else:
                if timeout_ros2 is not None:
                    raise ServiceException(
                        'Service did not return a response before timeout {}'.format(timeout_ros2))
                else:
                    raise ServiceException('Service did not return a response')

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
