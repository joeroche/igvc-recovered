import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


class PathTrailNode(Node):
    def __init__(self):
        super().__init__('path_trail_node')
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.publisher = self.create_publisher(Path, 'robot_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'robot_path_markers', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )
        self.velocity_subscription = self.create_subscription(
            Twist,
            'odrive/motor_velocity',
            self.velocity_callback,
            10
        )
        self.last_stamp = None
        self.left_velocity = 0.0
        self.right_velocity = 0.0

    def odom_callback(self, msg: Odometry) -> None:
        if self.last_stamp == msg.header.stamp:
            return

        self.last_stamp = msg.header.stamp
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)
        self.publisher.publish(self.path)

        self.publish_markers(msg)

    def velocity_callback(self, msg: Twist) -> None:
        self.left_velocity = msg.linear.x
        self.right_velocity = msg.linear.y

    def publish_markers(self, odom_msg: Odometry) -> None:
        marker_array = MarkerArray()

        current_position = odom_msg.pose.pose.position

        line_marker = Marker()
        line_marker.header = odom_msg.header
        line_marker.ns = 'robot_path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 0.0
        line_marker.color.g = 0.5
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        line_marker.pose.orientation.w = 1.0
        line_marker.points = [pose.pose.position for pose in self.path.poses]

        position_marker = Marker()
        position_marker.header = odom_msg.header
        position_marker.ns = 'robot_position'
        position_marker.id = 1
        position_marker.type = Marker.SPHERE
        position_marker.action = Marker.ADD
        position_marker.pose.position = current_position
        position_marker.pose.orientation.w = 1.0
        position_marker.scale.x = 0.2
        position_marker.scale.y = 0.2
        position_marker.scale.z = 0.2
        position_marker.color.r = 1.0
        position_marker.color.g = 0.0
        position_marker.color.b = 0.0
        position_marker.color.a = 0.8

        text_marker = Marker()
        text_marker.header = odom_msg.header
        text_marker.ns = 'odrive_velocity'
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = current_position
        text_marker.pose.position.z += 0.5
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = f'LV: {self.left_velocity:.2f}  RV: {self.right_velocity:.2f}'

        marker_array.markers.append(line_marker)
        marker_array.markers.append(position_marker)
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PathTrailNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
