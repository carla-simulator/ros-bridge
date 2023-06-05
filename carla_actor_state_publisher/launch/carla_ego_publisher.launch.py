import launch
import launch_ros.actions


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
            name='timeout',
            default_value='2'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch_ros.actions.Node(
            package='carla_actor_state_publisher',
            executable='carla_actor_state_publisher',
            name='carla_actor_state_publisher',
            output='screen',
            emulate_tty='True',
            parameters=[
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
