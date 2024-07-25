from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    line_follower_controller_node = Node(
        package='line_follower_controller',
        executable='line_follower',
        name='line_follower_node'
    )
    
    image_view_node = Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            arguments=['--ros-args', '--remap', 'image:=/processed_frame']
    )
    
    return LaunchDescription([
        line_follower_controller_node,
        image_view_node
    ])
