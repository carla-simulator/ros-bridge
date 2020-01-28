"""
Setup for carla_infrastructure
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['carla_infrastructure'],
    package_dir={'': 'src'}
)

setup(**d)
