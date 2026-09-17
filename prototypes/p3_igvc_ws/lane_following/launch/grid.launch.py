from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_grid',
            executable='pointcloud_to_grid_node',
            name='pointcloud_to_grid_node',
            output='screen',
            parameters=[{
                'cloud_in_topic': '/points',
                'height_layer': 'height',    
                'intensity_layer': '',       
                'publish_intensity_grid': False,
                'publish_height_grid': True,
            }]
        )
    ])
