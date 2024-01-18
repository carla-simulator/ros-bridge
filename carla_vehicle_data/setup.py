from setuptools import find_packages, setup

package_name = 'carla_vehicle_data'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hadhoum',
    maintainer_email='hadhoum.hajjaj@nist.gov',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'carla_vehicle_data = carla_vehicle_data.carla_vehicle_data:main',
        'carla_vehicle_pub = carla_vehicle_data.carla_vehicle_publisher:main',
        ],
    },
)
