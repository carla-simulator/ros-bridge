import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            LogInfo, RegisterEventHandler, IncludeLaunchDescription)
from launch.event_handlers import (OnExecutionComplete, OnProcessStart)
from launch.substitutions import (FindExecutable, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    timeout = LaunchConfiguration('timeout')
    role_name = LaunchConfiguration('role_name')
    vehicle_filter = LaunchConfiguration('vehicle_filter')
    spawn_point = LaunchConfiguration('spawn_point')
    town = LaunchConfiguration('town')

    load_map_call = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "/carla/load_map ",
            "carla_msgs/srv/LoadMap ",
            '"{mapname: "', town, '", force_reload: True}"',
        ]],
        shell=True
    )

    launch_carla_ros_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_ros_client'), 'carla_ros_client.launch.py')
        ),
        launch_arguments={
            'host': host,
            'port': port,
            'timeout': timeout
        }.items()
    )
    
    launch_ego_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'host': host,
            'port': port,
            'timeout': timeout,
            'vehicle_filter': vehicle_filter,
            'role_name': role_name,
            'spawn_point': spawn_point
        }.items()
    )

    launch_manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_manual_control'), 'carla_manual_control.launch.py')
        ),
        launch_arguments={
            'role_name': role_name
        }.items()
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.*'
        ),
        DeclareLaunchArgument(
            name='spawn_point',
            default_value='None'
        ),
        DeclareLaunchArgument(
            name='town',
            default_value='Town01'
        ),
#        load_map_call,
#        todo: fix event handler configuration (is not triggered as expected)
#        RegisterEventHandler(
#            OnExecutionComplete(
#                target_action=load_map_call,
#                on_completion=[
#                    LogInfo(msg="Map loaded, spawning vehicle"),
        launch_ego_vehicle,
#                    LogInfo(msg="Starting control nodes"),
        launch_carla_ros_client,
        launch_manual_control
#                ],
#            )
#        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
