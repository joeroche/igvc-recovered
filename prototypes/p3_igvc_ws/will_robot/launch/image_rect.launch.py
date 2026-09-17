from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'align_depth': True,
            'enable_sync': True,
            'enable_point_cloud':True,
        }]
    )

    image_proc_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_node',
                remappings=[
                    ('image_rect', 'image_rect_raw'),
                    ('camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
                    ('image','/camera/camera/aligned_depth_to_color/image_raw')
                ]
            )
        ],
        output='screen',
    )

    depth_proc_container = ComposableNodeContainer(
        name='depth_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', 'image_rect_raw'),
                    ('camera_info', '/camera/camera/aligned_depth_to_color/camera_info')
                ]
            )
        ],
        output='screen',
    )

    return LaunchDescription([realsense_node, image_proc_container, depth_proc_container])
