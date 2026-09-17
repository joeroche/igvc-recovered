from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth2grid',
            executable='depth2grid_node',
            name='depth2grid',
            output='screen',
            remappings=[
                ('/camera/depth/points', '/points')
            ],
        ),
    ])
