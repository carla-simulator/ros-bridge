# -*- coding: utf-8 -*-

"""
Setup for rqt_carla_control
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_carla_control'],
    package_dir={'': 'src'},
)

setup(**d)
