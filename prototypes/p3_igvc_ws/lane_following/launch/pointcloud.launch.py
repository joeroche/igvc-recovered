import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('/image_rect', '/camera/camera/depth/image_rect_raw'),
                        ('/camera_info', '/camera/camera/depth/camera_info')
                    ]
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])