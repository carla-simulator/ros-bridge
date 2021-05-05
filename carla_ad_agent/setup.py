"""
Setup for carla_ad_agent
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(packages=['carla_ad_agent'], package_dir={'': 'src'})

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'carla_ad_agent'
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            (os.path.join('share', package_name), ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='CARLA ROS2 AD agent',
        license='MIT',
        entry_points={
            'console_scripts': ['ad_agent = carla_ad_agent.ad_agent:main',
                                'local_planner = carla_ad_agent.local_planner:main',
                                'testing_waypoints = carla_ad_agent.testing_waypoints:main'],
        },
        package_dir={'': 'src'},
    )
