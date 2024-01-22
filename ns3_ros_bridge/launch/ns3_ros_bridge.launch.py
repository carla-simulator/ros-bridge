import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='delay_ms',
            default_value='0',
            description='Constant amount of additional delay to add to messages received from ns-3 (in milliseconds)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='stoptime',
            default_value='-1',
            description='Duration of the simulation based on the ROS2 Clock value (-1 to run forever)'
        ),
        launch_ros.actions.Node(
            package="ns3_ros_bridge",
            executable="ns3_ros_bridge",
            name="ns3_ros_bridge",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'delay_ms': launch.substitutions.LaunchConfiguration('delay_ms')
                },
                {
                    'stoptime': launch.substitutions.LaunchConfiguration('stoptime')
                }
            ]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
