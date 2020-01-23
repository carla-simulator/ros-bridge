# -*- coding: utf-8 -*-

"""
Setup for carla_waypoint_publisher
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_waypoint_publisher'],
    package_dir={'': 'src'},
)

setup(**d)
