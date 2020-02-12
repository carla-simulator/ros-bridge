"""
Setup for carla_spectator_camera
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_spectator_camera'],
    package_dir={'': 'src'}
)

setup(**d)
