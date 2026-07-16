import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'derived_objects_visualizer'

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
    description='DerivedObjects Visualizer',
    license='MIT',
    entry_points={
        'console_scripts': [
            f'derived_objects_visualizer = {package_name}.derived_objects_visualizer_main:main',
        ],
    },
)
