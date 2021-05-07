import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def launch_carla_spawn_object(context, *args, **kwargs):
    # workaround to use launch argument 'role_name' as a part of the string used for the spawn_point param name
    spawn_point_param_name = 'spawn_point_' + \
        launch.substitutions.LaunchConfiguration('role_name').perform(context)

    carla_spawn_objects_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': get_package_share_directory('carla_spawn_objects') + '/config/objects.json',
            spawn_point_param_name: launch.substitutions.LaunchConfiguration('spawn_point')
        }.items()
    )

    return [carla_spawn_objects_launch]

def launch_target_speed_publisher(context, *args, **kwargs):
    topic_name = "/carla/" + launch.substitutions.LaunchConfiguration('role_name').perform(context) + "/target_speed"
    data_string = "{'data': " + launch.substitutions.LaunchConfiguration('target_speed').perform(context) + "}"
    return [
        launch.actions.ExecuteProcess(
            output="screen",
            cmd=["ros2", "topic", "pub", topic_name,
                 "std_msgs/msg/Float64", data_string, "--qos-durability", "transient_local"],
            name='topic_pub_target_speed')]

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point',
            default_value='127.4,-195.4,2,0,0,180'
        ),
        launch.actions.DeclareLaunchArgument(
            name='target_speed',
            default_value='8.33' # in m/s
        ),
        launch.actions.DeclareLaunchArgument(
            name='avoid_risk',
            default_value='True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sigterm_timeout',
            default_value='15'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
            }.items()
        ),
        launch.actions.OpaqueFunction(function=launch_carla_spawn_object),
        launch.actions.OpaqueFunction(function=launch_target_speed_publisher),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ad_agent'), 'carla_ad_agent.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'avoid_risk': launch.substitutions.LaunchConfiguration('avoid_risk')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_waypoint_publisher'), 'carla_waypoint_publisher.launch.py')
            ),
            launch_arguments={
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
