import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(
            name='carla_host',
            default_value='localhost',
            description='IP address of the CARLA Simulator'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carla_port',
            default_value='2000',
            description='TCP port number of the CARLA Simulator'
        ),
        launch.actions.DeclareLaunchArgument(
            name='delay_ms',
            default_value='0',
            description='Constant amount of additional delay to add to messages received from ns-3 (in milliseconds)'
        ),
        launch_ros.actions.Node(
            package="ns3_ros_bridge",
            executable="bridge",
            name="ns3_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'carla_host': launch.substitutions.LaunchConfiguration('carla_host')
                },
                {
                    'carla_port': launch.substitutions.LaunchConfiguration('carla_port')
                },
                {
                    'delay_ms': launch.substitutions.LaunchConfiguration('delay_ms')
                }
            ]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
