#!/usr/bin/env python

#
# Copyright (c) 2021 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import ros_compatibility.qos
from ros_compatibility.core import get_ros_version
from ros_compatibility.exceptions import *

ROS_VERSION  = get_ros_version()

if ROS_VERSION == 1:

    import rospy

    from ros_compatibility.logging import logdebug, loginfo, logwarn, logerr, logfatal


    class CompatibleNode(object):

        def __init__(self, name, **kwargs):
           pass

        def get_param(self, name, alternative_value=None):
            if name.startswith('/'):
                raise RuntimeError("Only private parameters are supported.")
            return rospy.get_param("~" + name, alternative_value)

        def get_time(self):
            return rospy.get_time()

        def logdebug(self, msg):
            logdebug(msg)

        def loginfo(self, msg):
            loginfo(msg)

        def logwarn(self, msg):
            logwarn(msg)

        def logerr(self, msg):
            logerr(msg)

        def logfatal(self, msg):
            logfatal(msg)

        def new_publisher(self, msg_type, topic, qos_profile, callback_group=None):
            if isinstance(qos_profile, int):
                qos_profile = ros_compatibility.qos.QoSProfile(depth=qos_profile)

            return rospy.Publisher(topic, msg_type, queue_size=qos_profile.depth, latch=qos_profile.is_latched())

        def new_subscription(self, msg_type, topic, callback, qos_profile, callback_group=None):
            if isinstance(qos_profile, int):
                qos_profile = ros_compatibility.qos.QoSProfile(depth=qos_profile)

            return rospy.Subscriber(topic, msg_type, callback, queue_size=qos_profile.depth)

        def new_rate(self, frequency):
            return rospy.Rate(frequency)

        def new_timer(self, timer_period_sec, callback, callback_group=None):
            return rospy.Timer(rospy.Duration(timer_period_sec), callback)

        def wait_for_message(self, topic, msg_type, timeout=None, qos_profile=None):
            try:
                return rospy.wait_for_message(topic, msg_type, timeout)
            except rospy.ROSException as e:
                raise ROSException(e)

        def new_service(self, srv_type, srv_name, callback, qos_profile=None, callback_group=None):
            return rospy.Service(srv_name, srv_type, callback)

        def new_client(self, srv_type, srv_name, timeout_sec=None, callback_group=None):
            if timeout_sec is not None:
                timeout = timeout_sec * 1000
            else:
                timeout = timeout_sec
            try:
                rospy.wait_for_service(srv_name, timeout=timeout)
                client = rospy.ServiceProxy(srv_name, srv_type)
            except rospy.ServiceException as e:
                raise ServiceException(e)
            except rospy.ROSException as e:
                raise ROSException(e)
            return client

        def call_service(self, client, req, timeout=None, spin_until_response_received=False):
            try:
                return client(req)
            except rospy.ServiceException as e:
                raise ServiceException(e)

        def spin(self):
            rospy.spin()

        def destroy_service(self, service):
            service.shutdown()

        def destroy_subscription(self, subscription):
            subscription.unregister()

        def destroy_publisher(self, publisher):
            publisher.unregister()

        def destroy(self):
            pass

elif ROS_VERSION == 2:

    import time
    from rclpy import Parameter
    from rclpy.node import Node
    from rclpy.task import Future
    import rclpy.qos

    _DURABILITY_POLICY_MAP = {
        ros_compatibility.qos.DurabilityPolicy.TRANSIENT_LOCAL: rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        ros_compatibility.qos.DurabilityPolicy.VOLATILE: rclpy.qos.DurabilityPolicy.VOLATILE
    }


    def _get_rclpy_qos_profile(qos_profile):
        return rclpy.qos.QoSProfile(
            depth=qos_profile.depth,
            durability=_DURABILITY_POLICY_MAP[qos_profile.durability]
        )


    class CompatibleNode(Node):

        def __init__(self, name, **kwargs):
            param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
            super(CompatibleNode, self).__init__(
                name,
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True,
                parameter_overrides=[param],
                **kwargs)

        def get_param(self, name, alternative_value=None):
            return self.get_parameter_or(
                name,
                Parameter(name, value=alternative_value)).value

        def get_time(self):
            t = self.get_clock().now()
            t = t.seconds_nanoseconds()
            return float(t[0] + (t[1] / 10**9))

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

        def new_publisher(self, msg_type, topic, qos_profile, callback_group=None):
            if isinstance(qos_profile, int):
                qos_profile = ros_compatibility.qos.QoSProfile(depth=qos_profile)
            rclpy_qos_profile = _get_rclpy_qos_profile(qos_profile)

            return self.create_publisher(
                msg_type, topic, rclpy_qos_profile,
                callback_group=callback_group)

        def new_subscription(self, msg_type, topic, callback, qos_profile, callback_group=None):
            if isinstance(qos_profile, int):
                qos_profile = ros_compatibility.qos.QoSProfile(depth=qos_profile)
            rclpy_qos_profile = _get_rclpy_qos_profile(qos_profile)

            return self.create_subscription(
                msg_type, topic, callback, rclpy_qos_profile,
                callback_group=callback_group)

        def new_rate(self, frequency):
            return self.create_rate(frequency)

        def new_timer(self, timer_period_sec, callback, callback_group=None):
            return self.create_timer(
                timer_period_sec, callback, callback_group=callback_group)

        def wait_for_message(self, topic, msg_type, timeout=None, qos_profile=1):
            """
            Wait for one message from topic.
            This will create a new subcription to the topic, receive one message, then unsubscribe.
 
            Do not call this method in a callback or a deadlock may occur.
            """
            s = None
            try:
                future = Future()
                s = self.new_subscription(
                    msg_type,
                    topic,
                    lambda msg: future.set_result(msg),
                    qos_profile=qos_profile)
                rclpy.spin_until_future_complete(self, future, self.executor, timeout)
            finally:
                if s is not None:
                    self.destroy_subscription(s)

            return future.result()

        def new_service(self, srv_type, srv_name, callback, callback_group=None):
            return self.create_service(
                srv_type, srv_name, callback, callback_group=callback_group)

        def new_client(self, srv_type, srv_name, timeout_sec=None, callback_group=None):
            client = self.create_client(
                srv_type, srv_name, callback_group=callback_group)
            ready = client.wait_for_service(timeout_sec=timeout_sec)
            if not ready:
                raise ROSException("Timeout of {}sec while waiting for service".format(timeout_sec))
            return client

        def call_service(self, client, req, timeout=None, spin_until_response_received=False):
            if not spin_until_response_received:
                response = client.call(req)
                return response
            else:
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, self.executor, timeout)

                if future.done():
                    return future.result()
                else:
                    if timeout is not None:
                        raise ServiceException(
                            'Service did not return a response before timeout {}'.format(timeout))
                    else:
                        raise ServiceException('Service did not return a response')

        def spin(self):
            rclpy.spin(self, self.executor)

        def destroy(self):
            self.destroy_node()
