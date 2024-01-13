"""
Setup for carla_twist_to_control
"""

import os
from glob import glob
from setuptools import setup
ROS_VERSION = int(os.environ['ROS_VERSION'])

package_name = 'carla_twist_to_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name), glob('launch/*.launch.py'))
              ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='CARLA twist to control for ROS2 bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
      'console_scripts': ['carla_twist_to_control = carla_twist_to_control.carla_twist_to_control:main'],
    },
    package_dir={'': 'src'},
    package_data={'': ['CARLA_VERSION']},
)
