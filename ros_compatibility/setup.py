import os

ROS_VERSION = int(os.environ.get('ROS_VERSION', 0))

from setuptools import setup

package_name = 'ros_compatibility'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml'])],
    # py_modules=[
    #     'src.ros_compatibility.ros_compatible_node'],
    #     data_files=[
    #     ('share/ament_index/resource_index/packages',
    #      ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='The ros_compatibility package',
    license='MIT',
    tests_require=['pytest'],
    package_dir={'': 'src'},
)
