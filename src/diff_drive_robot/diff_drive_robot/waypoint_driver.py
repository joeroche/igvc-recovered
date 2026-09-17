#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointDriver(Node):
    def __init__(self):
        super().__init__('waypoint_driver')

        self.declare_parameter('goal_frame', 'odom')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        # ROS 2 parameter YAML cannot reliably represent a list of dictionaries
        # in Humble's rcl parser, so use parallel numeric arrays instead.
        self.declare_parameter('goal_xs', [1.0, 1.0, 0.0])
        self.declare_parameter('goal_ys', [0.0, 1.0, 1.0])
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('yaw_gain', 1.5)
        self.declare_parameter('pause_at_goal', 2.0)

        self.goal_frame = self.get_parameter('goal_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        goal_xs = list(self.get_parameter('goal_xs').value)
        goal_ys = list(self.get_parameter('goal_ys').value)
        if len(goal_xs) != len(goal_ys):
            raise ValueError(
                f'goal_xs and goal_ys must have the same length: '
                f'{len(goal_xs)} != {len(goal_ys)}')
        self.goals = [
            {'x': float(x), 'y': float(y)}
            for x, y in zip(goal_xs, goal_ys)
        ]
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.pause_at_goal = self.get_parameter('pause_at_goal').value

        self.current_goal_index = 0
        self.current_pose = None
        self.pause_until = None

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/waypoint_cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Waypoint driver started with {len(self.goals)} goals, '
            f'odom_topic={self.odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.current_pose is None or self.current_goal_index >= len(self.goals):
            return

        if self.pause_until is not None:
            if self.get_clock().now().nanoseconds / 1e9 < self.pause_until:
                return
            self.pause_until = None

        goal = self.goals[self.current_goal_index]
        dx = goal['x'] - self.current_pose.position.x
        dy = goal['y'] - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        output = Twist()

        if distance < self.goal_tolerance:
            self.get_logger().info('Reached goal %d at (%.2f, %.2f)', self.current_goal_index, goal['x'], goal['y'])
            self.current_goal_index += 1
            self.pause_until = self.get_clock().now().nanoseconds / 1e9 + self.pause_at_goal
            self.cmd_pub.publish(output)
            return

        target_yaw = math.atan2(dy, dx)
        current_yaw = quaternion_to_yaw(self.current_pose.orientation)
        yaw_error = wrap_angle(target_yaw - current_yaw)

        angular_cmd = max(min(self.yaw_gain * yaw_error, self.max_angular_speed), -self.max_angular_speed)
        forward_speed = self.max_speed * min(distance / 1.0, 1.0)

        if abs(yaw_error) > 0.35:
            output.linear.x = 0.0
        else:
            output.linear.x = forward_speed

        output.angular.z = angular_cmd
        self.cmd_pub.publish(output)

    def get_current_goal(self):
        return self.goals[self.current_goal_index] if self.current_goal_index < len(self.goals) else None


def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
