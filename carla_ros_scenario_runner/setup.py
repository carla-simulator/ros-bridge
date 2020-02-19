"""
Setup for carla_ros_scenario_runner
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_ros_scenario_runner'],
    package_dir={'': 'src'}
)

setup(**d)
