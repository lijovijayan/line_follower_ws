from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('line_follower_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'line_follower.xacro')
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

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    urdf_spawn_node = Node(
        package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', robot_urdf,
                '-name', LaunchConfiguration('robot_name'),
                '-x', LaunchConfiguration('initial_pose_x'),
                '-y', LaunchConfiguration('initial_pose_y'),
                '-z', LaunchConfiguration('initial_pose_z'),
                '-Y', LaunchConfiguration('initial_pose_yaw'),
            ],
            output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name',
            default_value='my_robot',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_pose_x',
            default_value='0.0',
            description='Initial x position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_pose_y',
            default_value='0.0',
            description='Initial y position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_pose_z',
            default_value='0.0',
            description='Initial z position of the robot'
        ),
        DeclareLaunchArgument(
            name='initial_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation of the robot'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        urdf_spawn_node,
    ])
