import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'carla_map_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CARLA Simulator Team',
    maintainer_email='carla.simulator@gmail.com',
    description='CARLA Map Visualizer',
    license='MIT',
    entry_points={
        'console_scripts': [
            f'carla_map_visualizer = {package_name}.carla_map_visualizer_main:main',
        ],
    },
)
