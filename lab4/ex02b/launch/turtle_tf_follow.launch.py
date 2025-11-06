from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='2.0', description='Радиус вращения морковки'),
        DeclareLaunchArgument('direction_of_rotation', default_value='1', description='Направление вращения (1 или -1)'),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        Node(
            package='turtle_tf_follow',
            executable='carrot_tf_broadcaster',
            name='carrot_tf_broadcaster',
            output='screen',
            parameters=[
                {'radius': LaunchConfiguration('radius')},
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
            ]
        ),
        Node(
            package='turtle_tf_follow',
            executable='follower',
            name='follower',
            output='screen'
        ),
        Node(
            package='turtle_tf_follow',
            executable='turtlesim_tf_broadcaster',
            name='turtlesim_tf_broadcaster',
            output='screen'
        ),
    ])
