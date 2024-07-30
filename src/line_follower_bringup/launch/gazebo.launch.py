from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share_dir = get_package_share_directory('line_follower_description')
    worlds_share_dir = get_package_share_directory('line_follower_worlds')
    world_file_path = os.path.join(worlds_share_dir, 'worlds', 'line_follower.world')
    # world_file_path = os.path.join(worlds_share_dir, 'worlds', 'empty_world.world')

    # xacro_file = os.path.join(description_share_dir, 'sample_urdf', 'line_follower.sample_urdf.xacro')
    xacro_file = os.path.join(description_share_dir, 'robot_urdf', 'line_follower.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_file_path,
            'pause': 'true'
        }.items()
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'line_follower',
            '-topic', 'robot_description',
            '-x', '-17.0',
            '-y', '17.765076',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-1.5708'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        urdf_spawn_node
    ])
