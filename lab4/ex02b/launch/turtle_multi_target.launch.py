from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('switch_threshold', default_value='1.0', 
                            description='Distance threshold for auto-switching targets'),
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),
        
        Node(
            package='turtle_tf_follow',
            executable='turtlesim_tf_broadcaster',
            name='turtlesim_tf_broadcaster',
            output='screen'
        ),
        
        Node(
            package='turtle_tf_follow',
            executable='target_switcher',
            name='target_switcher',
            output='screen',
            parameters=[
                {'switch_threshold': LaunchConfiguration('switch_threshold')}
            ]
        ),
        
        Node(
            package='turtle_tf_follow',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen',
            parameters=[
                {'switch_threshold': LaunchConfiguration('switch_threshold')}
            ]
        ),
    ])
