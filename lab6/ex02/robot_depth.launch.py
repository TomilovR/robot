import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot = get_package_share_directory('robot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(pkg_robot, 'zhiguli_gazebo.urdf')
    rviz_config = os.path.join(pkg_robot, 'urdf.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
        ),
        launch_arguments={
            "gz_args": "-r gpu_lidar_sensor.sdf"
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'zhiguli',
            '-string', robot_desc,
            '-x', '0', '-y', '2.0', '-z', '0.5'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/gpu_lidar_sensor/model/zhiguli/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        remappings=[
            ('/world/gpu_lidar_sensor/model/zhiguli/joint_state', '/joint_states')
        ],
        output='screen'
    )

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'zhiguli/base_link/gpu_lidar'],
        output='screen'
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'zhiguli/camera_link/depth_camera'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        robot_state_publisher,
        rviz,
        bridge,
        static_tf_lidar,
        static_tf_camera
    ])
