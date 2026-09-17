#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import math


class BehaviorNode(Node):

    def __init__(self):
        super().__init__('behavior_node')

        # -------- PARAMETERS --------
        self.declare_parameter('obstacle_distance', 1.2)
        self.declare_parameter('avoid_strength', 1.5)
        self.declare_parameter('front_avoid_strength', 1.0)
        self.declare_parameter('forward_speed_scale', 0.6)
        self.declare_parameter('follow_left', True)
        self.declare_parameter('critical_distance', 1.5)
        self.declare_parameter('use_map_avoidance', False)

        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.avoid_strength = self.get_parameter('avoid_strength').value
        self.front_avoid_strength = self.get_parameter('front_avoid_strength').value
        self.forward_scale = self.get_parameter('forward_speed_scale').value
        self.follow_left = self.get_parameter('follow_left').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.use_map_avoidance = self.get_parameter('use_map_avoidance').value

        # -------- STATE --------
        self.latest_line_cmd = Twist()
        self.latest_obstacle_cmd = Twist()
        self.critical_obstacle_detected = False
        self.turning_away = False

        # -------- SUBSCRIBERS --------
        self.line_sub = self.create_subscription(
            Twist,
            '/line_cmd_vel',
            self.line_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            Twist,
            '/obstacle_cmd_vel',
            self.obstacle_callback,
            10
        )

        self.waypoint_sub = self.create_subscription(
            Twist,
            '/waypoint_cmd_vel',
            self.waypoint_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # -------- PUBLISHERS --------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )

        self.sim_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # waypoint navigation command
        self.latest_waypoint_cmd = Twist()

        # obstacle influence
        self.avoid_angular = 0.0
        self.slowdown_factor = 1.0
        self.map_avoid_angular = 0.0
        self.occupancy_grid = None

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Behavior node (lane + obstacle avoidance) started")

    # -------- LINE INPUT --------
    def line_callback(self, msg):
        self.latest_line_cmd = msg

    # -------- OBSTACLE INPUT --------
    def obstacle_callback(self, msg):
        self.latest_obstacle_cmd = msg

    # -------- WAYPOINT INPUT --------
    def waypoint_callback(self, msg):
        self.latest_waypoint_cmd = msg

    # -------- MAP PROCESSING --------
    def map_callback(self, msg):
        """Subscribe to occupancy grid and detect high-cost areas to avoid"""
        self.occupancy_grid = msg
        self.map_avoid_angular = 0.0

        # Check cells in front of robot for high occupancy (previously explored areas)
        if msg.data and len(msg.data) > 0:
            # Simple heuristic: check central region of the map
            grid_width = msg.info.width
            center_x = grid_width // 2
            
            # Check cells at different angles ahead
            left_cost = 0
            right_cost = 0
            
            # Sample left side (indices ahead and to the left)
            for i in range(max(0, center_x - 20), max(0, center_x - 5)):
                for j in range(len(msg.data)):
                    if j < len(msg.data) and msg.data[j] > 70:  # High occupancy threshold
                        left_cost += 1
            
            # Sample right side (indices ahead and to the right)
            for i in range(min(grid_width, center_x + 5), min(grid_width, center_x + 20)):
                for j in range(len(msg.data)):
                    if j < len(msg.data) and msg.data[j] > 70:  # High occupancy threshold
                        right_cost += 1
            
            # If left side has more obstacles, turn right and vice versa
            if left_cost > right_cost and left_cost > 10:
                self.map_avoid_angular = 0.3  # Turn right
            elif right_cost > left_cost and right_cost > 10:
                self.map_avoid_angular = -0.3  # Turn left

    # -------- LIDAR PROCESSING --------
    def scan_callback(self, msg):

        left_force = 0.0
        right_force = 0.0
        front_block = False
        critical_front_block = False

        angle = msg.angle_min

        for r in msg.ranges:

            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            # Check for critical obstacles within 1.5 meters
            if r < self.critical_distance:
                print(f"Critical obstacle detected at distance {r:.2f}m and angle {math.degrees(angle):.1f}°")
                # FRONT critical zone
                if -0.3 < angle < 0.3:
                    critical_front_block = True

            # Only care about nearby obstacles for normal avoidance
            if r < self.obstacle_distance:
                print(f"Obstacle detected at distance {r:.2f}m and angle {math.degrees(angle):.1f}°")

                strength = (self.obstacle_distance - r) / self.obstacle_distance

                # FRONT
                if -0.3 < angle < 0.3:
                    print("Obstacle in front - applying strong avoidance")
                    front_block = True

                # LEFT SIDE
                elif angle > 0:
                    print("Obstacle on left side - applying avoidance")
                    left_force += strength

                # RIGHT SIDE
                else:
                    right_force += strength

            angle += msg.angle_increment

        # Set critical obstacle flag
        self.critical_obstacle_detected = critical_front_block

        # If critical obstacle detected, start turning away from followed side
        if self.critical_obstacle_detected and not self.turning_away:
            print("Critical obstacle detected - initiating turn away from followed side")
            
            self.turning_away = True
            self.get_logger().info("Critical obstacle detected - turning away from followed side")
            print("critical obstacle detected - turning away from followed side")

        # If no longer critical obstacle ahead, stop turning away
        if not critical_front_block and self.turning_away:
            self.turning_away = False
            self.get_logger().info("Clear ahead - resuming normal behavior")
            print("clear ahead - resuming normal behavior")

        # Normal obstacle avoidance steering
        self.avoid_angular = (right_force - left_force) * self.avoid_strength

        # If an obstacle is directly ahead, bias turn away from the busiest side
        if front_block:
            if left_force > right_force:
                self.avoid_angular += -self.front_avoid_strength
            else:
                self.avoid_angular += self.front_avoid_strength

        # Slow down if obstacle ahead
        self.slowdown_factor = 0.3 if front_block else 1.0

    # -------- CONTROL LOOP --------
    def control_loop(self):

        output = Twist()

        # Priority order: Lidar critical > Camera obstacles > Map avoidance > Line following

        # Check for critical lidar obstacle behavior (highest priority)
        if self.turning_away:
            # Turn away from the followed side and towards the other side
            # If following left, turn right (positive angular.z)
            # If following right, turn left (negative angular.z)
            turn_direction = 1.0 if self.follow_left else -1.0
            output.angular.z = turn_direction * 1.5  # Strong turning
            output.linear.x = 0.0  # Stop forward motion while turning
            self.get_logger().debug("Critical lidar obstacle - turning away from followed side")

        # Check for camera-detected large obstacles (medium priority)
        elif abs(self.latest_obstacle_cmd.angular.z) > 0.01 or abs(self.latest_obstacle_cmd.linear.x) > 0.01:
            output.linear.x = self.latest_obstacle_cmd.linear.x
            output.angular.z = self.latest_obstacle_cmd.angular.z
            self.get_logger().debug("Camera obstacle avoidance active")

        # Check for waypoint navigation target (medium priority)
        elif abs(self.latest_waypoint_cmd.angular.z) > 0.01 or abs(self.latest_waypoint_cmd.linear.x) > 0.01:
            output.linear.x = self.latest_waypoint_cmd.linear.x
            output.angular.z = self.latest_waypoint_cmd.angular.z
            self.get_logger().debug("Waypoint navigation active")

        # Check for SLAM map avoidance (medium-low priority)
        elif self.use_map_avoidance and abs(self.map_avoid_angular) > 0.05:
            output.linear.x = self.latest_line_cmd.linear.x * self.forward_scale * self.slowdown_factor * 0.8
            output.angular.z = self.map_avoid_angular
            self.get_logger().debug("Avoiding previously mapped areas")

        else:
            # Base: lane following (lowest priority)
            output.linear.x = self.latest_line_cmd.linear.x * self.forward_scale * self.slowdown_factor
            output.angular.z = self.latest_line_cmd.angular.z

            # Add avoidance (this is the KEY part)
            output.angular.z += self.avoid_angular

        # Clamp turning (prevents instability)
        output.angular.z = max(min(output.angular.z, 2.0), -2.0)

        self.cmd_pub.publish(output)
        self.sim_cmd_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
