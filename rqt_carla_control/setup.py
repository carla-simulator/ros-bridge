#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Setup for rqt_carla_control
"""
import os
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['rqt_carla_control'],
        package_dir={'': 'src'},
    )

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'rqt_carla_control'
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        package_dir={'': 'src'},
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            (os.path.join('share', package_name), ['package.xml']),
            (os.path.join('share', package_name), ['plugin.xml']),
            ('share/' + package_name + '/resource',
             ['resource/CarlaControl.ui', 'resource/pause.png', 'resource/play.png', 'resource/step_once.png']),
            ('lib/' + package_name, ['scripts/rqt_carla_control'])
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='CARLA ROS2 RQT CONTROL',
        license='MIT',
        scripts=['scripts/rqt_carla_control'],
    )
