"""
Setup for carla_ros_client
"""
import os
from glob import glob
from setuptools import setup

package_name = 'carla_ros_client'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name), glob('launch/*.launch.py'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='CARLA ROS2 client',
    license='MIT',
    entry_points={
        'console_scripts': ['client = carla_ros_client.client:main'],
    },
    package_dir={'': 'src'},
    package_data={'': ['CARLA_VERSION']},
    include_package_data=True
)
