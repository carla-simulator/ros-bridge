#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
RQT Plugin to control CARLA
"""
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget  # pylint: disable=no-name-in-module, import-error
from python_qt_binding.QtGui import QPixmap, QIcon  # pylint: disable=no-name-in-module, import-error

from carla_msgs.msg import CarlaControl, CarlaStatus


class CarlaControlPlugin(Plugin):

    """
    RQT Plugin to control CARLA
    """

    def __init__(self, context):
        """
        Constructor
        """
        super(CarlaControlPlugin, self).__init__(context)
        self.setObjectName('CARLA Control')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'rqt_carla_control'), 'resource', 'CarlaControl.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('CarlaControl')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.pause_icon = QIcon(
            QPixmap(os.path.join(
                rospkg.RosPack().get_path('rqt_carla_control'), 'resource', 'pause.png')))
        self.play_icon = QIcon(
            QPixmap(os.path.join(
                rospkg.RosPack().get_path('rqt_carla_control'), 'resource', 'play.png')))
        self._widget.pushButtonStepOnce.setIcon(
            QIcon(QPixmap(os.path.join(
                rospkg.RosPack().get_path('rqt_carla_control'), 'resource', 'step_once.png'))))

        self.carla_status = None
        self.carla_status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self.carla_status_changed)
        self.carla_control_publisher = rospy.Publisher(
            "/carla/control", CarlaControl, queue_size=10)

        self._widget.pushButtonPlayPause.setDisabled(True)
        self._widget.pushButtonStepOnce.setDisabled(True)
        self._widget.pushButtonPlayPause.setIcon(self.play_icon)
        self._widget.pushButtonPlayPause.clicked.connect(self.toggle_play_pause)
        self._widget.pushButtonStepOnce.clicked.connect(self.step_once)

        context.add_widget(self._widget)

    def toggle_play_pause(self):
        """
        toggle play/pause
        """
        if self.carla_status.synchronous_mode:
            if self.carla_status.synchronous_mode_running:
                self.carla_control_publisher.publish(CarlaControl(command=CarlaControl.PAUSE))
            else:
                self.carla_control_publisher.publish(CarlaControl(command=CarlaControl.PLAY))

    def step_once(self):
        """
        execute one step
        """
        self.carla_control_publisher.publish(CarlaControl(command=CarlaControl.STEP_ONCE))

    def carla_status_changed(self, status):
        """
        callback whenever carla status changes
        """
        self.carla_status = status
        if status.synchronous_mode:
            self._widget.pushButtonPlayPause.setDisabled(False)
            self._widget.pushButtonStepOnce.setDisabled(False)
            if status.synchronous_mode_running:
                self._widget.pushButtonPlayPause.setIcon(self.pause_icon)
            else:
                self._widget.pushButtonPlayPause.setIcon(self.play_icon)
        else:
            self._widget.pushButtonPlayPause.setDisabled(True)
            self._widget.pushButtonStepOnce.setDisabled(True)

    def shutdown_plugin(self):
        """
        shutdown plugin
        """
        self.carla_control_publisher.unregister()
