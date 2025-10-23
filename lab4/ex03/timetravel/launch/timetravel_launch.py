from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    delay_arg = DeclareLaunchArgument('delay', default_value='2.0', description='Delay in seconds')

    turtlesim = ExecuteProcess(
        cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
        output='screen'
    )

    spawn_call = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
            "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}",
        ],
        output='screen'
    )

    spawn_timed = TimerAction(
        period=1.0,
        actions=[spawn_call],
    )

    follower = Node(
        package='timetravel',
        executable='follower',
        name='time_travel_follower',
        output='screen',
        parameters=[{'delay': LaunchConfiguration('delay')}]
    )

    ld = LaunchDescription()
    ld.add_action(delay_arg)
    ld.add_action(turtlesim)
    ld.add_action(spawn_timed)
    ld.add_action(TimerAction(period=1.5, actions=[follower]))

    return ld
