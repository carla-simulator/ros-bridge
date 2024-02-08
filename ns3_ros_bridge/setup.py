import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ns3_ros_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas Roth',
    maintainer_email='thomas.roth@nist.gov',
    description='Connect ns-3 to ROS2 using TCP/IP',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = ns3_ros_bridge.bridge:main'
        ],
    },
)
