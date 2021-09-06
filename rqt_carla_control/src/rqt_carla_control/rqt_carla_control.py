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
import threading

from python_qt_binding import loadUi  # pylint: disable=import-error
from python_qt_binding.QtGui import QPixmap, QIcon  # pylint: disable=no-name-in-module, import-error
from python_qt_binding.QtWidgets import QWidget  # pylint: disable=no-name-in-module, import-error
from qt_gui.plugin import Plugin  # pylint: disable=import-error

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from carla_msgs.msg import CarlaControl, CarlaStatus  # pylint: disable=import-error


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

        self._node = CompatibleNode('rqt_carla_control_node')

        package_share_dir = roscomp.get_package_share_directory('rqt_carla_control')
        ui_file = os.path.join(package_share_dir, 'resource', 'CarlaControl.ui')

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('CarlaControl')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.pause_icon = QIcon(
            QPixmap(os.path.join(
                package_share_dir, 'resource', 'pause.png')))
        self.play_icon = QIcon(
            QPixmap(os.path.join(
                package_share_dir, 'resource', 'play.png')))
        self._widget.pushButtonStepOnce.setIcon(
            QIcon(QPixmap(os.path.join(
                package_share_dir, 'resource', 'step_once.png'))))

        self.carla_status = None
        self.carla_status_subscriber = self._node.new_subscription(
            CarlaStatus,
            "/carla/status",
            self.carla_status_changed,
            qos_profile=10)

        self.carla_control_publisher = self._node.new_publisher(
            CarlaControl,
            "/carla/control",
            qos_profile=10)

        self._widget.pushButtonPlayPause.setDisabled(True)
        self._widget.pushButtonStepOnce.setDisabled(True)
        self._widget.pushButtonPlayPause.setIcon(self.play_icon)
        self._widget.pushButtonPlayPause.clicked.connect(self.toggle_play_pause)
        self._widget.pushButtonStepOnce.clicked.connect(self.step_once)

        context.add_widget(self._widget)

        if roscomp.get_ros_version() == 2:
            spin_thread = threading.Thread(target=self._node.spin, daemon=True)
            spin_thread.start()

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
        self._node.destroy_subscription(self.carla_control_publisher)
