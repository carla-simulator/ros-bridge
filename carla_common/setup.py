"""
Setup for carla_common
"""

import os
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        packages=['carla_common'],
        package_dir={'': 'src'}
    )

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = 'carla_common'
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='CARLA Simulator Team',
        maintainer_email='carla.simulator@gmail.com',
        description='CARLA common providing transforms for ROS2 bridge',
        license='MIT',
        tests_require=['pytest'],
        package_dir={'': 'src'},
    )
