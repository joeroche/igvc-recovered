#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectLine(Node):

    def __init__(self):
        super().__init__('detect_line_node')

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('base_speed', 1.2)
        self.declare_parameter('kp', 0.004)
        self.declare_parameter('lookahead_gain', 1.5)
        self.declare_parameter('angle_gain', 0.5)
        self.declare_parameter('follow_left', True)
        self.declare_parameter('min_contour_area', 300)
        self.declare_parameter('min_padding', 80)

        self.declare_parameter('search_speed', 0.3)
        self.declare_parameter('search_turn', -0.3)
        self.declare_parameter('line_lost_time', 0.5)

        # HSV tuning params — adjust these first if line detection is wrong
        self.declare_parameter('white_v_min', 170)   # CHANGED: was hardcoded 200, too strict
        self.declare_parameter('white_s_max', 55)    # CHANGED: was hardcoded 60

        self.image_topic    = self.get_parameter('image_topic').value
        self.base_speed     = self.get_parameter('base_speed').value
        self.kp             = self.get_parameter('kp').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.angle_gain     = self.get_parameter('angle_gain').value
        self.follow_left    = self.get_parameter('follow_left').value
        self.min_area       = self.get_parameter('min_contour_area').value
        self.min_padding    = self.get_parameter('min_padding').value
        self.search_speed   = self.get_parameter('search_speed').value
        self.search_turn    = self.get_parameter('search_turn').value
        self.line_lost_time = self.get_parameter('line_lost_time').value
        self.white_v_min    = self.get_parameter('white_v_min').value
        self.white_s_max    = self.get_parameter('white_s_max').value

        # CHANGED: target_ratio moved inward (0.35/0.65 instead of 0.25/0.75)
        # so the error term isn't huge when the line is near the edge
        self.target_ratio = 0.35 if self.follow_left else 0.65

        self.last_line_time      = self.get_clock().now().nanoseconds * 1e-9
        self.search_mode         = False
        self.recovery_mode       = False
        self.both_visible_frames = 0

        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)

        self.obstacle_sub = self.create_subscription(
            Twist, '/obstacle_cmd_vel', self.obstacle_callback, 10)

        self.cmd_pub      = self.create_publisher(Twist, '/line_cmd_vel',        10)
        self.filtered_pub = self.create_publisher(Image, '/filtered_white_mask', 10)
        self.obstacle_pub = self.create_publisher(Twist, '/obstacle_cmd_vel',    10)

        self.get_logger().info(
            f'DetectLine ready | follow_left={self.follow_left} | '
            f'white HSV V>={self.white_v_min} S<={self.white_s_max}'
        )

    def get_nearest_contour(self, contours):
        best   = None
        best_y = -1
        for c in contours:
            if cv2.contourArea(c) < self.min_area:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cy = int(M["m01"] / M["m00"])
            if cy > best_y:
                best_y = cy
                best   = c
        return best

    def get_centroid_x(self, binary_img):
        contours, _ = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        nearest = self.get_nearest_contour(contours)
        if nearest is None:
            return None, None
        M = cv2.moments(nearest)
        if M["m00"] == 0:
            return None, None
        cx = int(M["m10"] / M["m00"])
        return cx, nearest

    def obstacle_callback(self, msg):
        if abs(msg.angular.z) > 0.01 or abs(msg.linear.x) > 0.01:
            self.get_logger().info('Obstacle avoidance active - entering recovery mode')
            self.recovery_mode       = True
            self.both_visible_frames = 0
        else:
            self.recovery_mode = False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]

        roi = frame[int(h * 0.25):int(h * 0.85), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # CHANGED: use tunable params instead of hardcoded values,
        # and add a second range to catch off-white / shadowed line segments
        lower_white = np.array([0,  0,              self.white_v_min])
        upper_white = np.array([180, self.white_s_max, 255])
        mask_bright = cv2.inRange(hsv, lower_white, upper_white)

        # Second range: lower value floor, tighter saturation — catches shadows
        lower_offwhite = np.array([0,  0,  max(140, self.white_v_min - 30)])
        upper_offwhite = np.array([180, 25, self.white_v_min - 1])
        mask_shadow    = cv2.inRange(hsv, lower_offwhite, upper_offwhite)

        mask = cv2.bitwise_or(mask_bright, mask_shadow)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Publish the filtered white mask
        self.filtered_pub.publish(
            self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        # Log white coverage so you can diagnose threshold issues
        white_pct = cv2.countNonZero(mask) / (mask.shape[0] * w) * 100
        if white_pct < 1.0:
            self.get_logger().warn(
                f'Only {white_pct:.1f}% white detected — '
                'try lowering white_v_min or raising white_s_max')
        else:
            self.get_logger().debug(f'White coverage: {white_pct:.1f}%')

        h_roi = mask.shape[0]
        near  = mask[int(h_roi * 0.6):, :]
        far   = mask[:int(h_roi * 0.4), :]

        cx_near, contour_near = self.get_centroid_x(near)
        cx_far,  _            = self.get_centroid_x(far)

        twist = Twist()
        now   = self.get_clock().now().nanoseconds * 1e-9

        # Recovery mode: only resume when both near and far are visible
        if self.recovery_mode:
            if cx_near is not None and cx_far is not None:
                self.both_visible_frames += 1
                if self.both_visible_frames >= 3:
                    self.recovery_mode = False
                    self.get_logger().info('Recovery complete - both lines visible')
            else:
                self.both_visible_frames = 0

        if cx_near is not None:
            self.last_line_time = now
            self.search_mode    = False

            target_x   = int(w * self.target_ratio)
            error_near = cx_near - target_x
            error_far  = 0 if cx_far is None else (cx_far - target_x)

            steering = (-self.kp * error_near) - (self.lookahead_gain * self.kp * error_far)

            # CHANGED: angle correction now normalised to [-pi/2, pi/2]
            # to avoid the sign flip fitLine can produce on ambiguous contours
            if contour_near is not None:
                [vx, vy, _, _] = cv2.fitLine(
                    contour_near, cv2.DIST_L2, 0, 0.01, 0.01)
                angle = float(np.arctan2(float(vy), float(vx)))
                if angle >  np.pi / 2: angle -= np.pi
                if angle < -np.pi / 2: angle += np.pi
                steering += -self.angle_gain * angle

            # CHANGED: padding nudge reduced from 0.6 → 0.25 to avoid overcorrection
            if self.follow_left  and cx_near < self.min_padding:
                steering += 0.25
            if not self.follow_left and cx_near > (w - self.min_padding):
                steering -= 0.25

            # CHANGED: removed hardcoded -0.03 bias; tune steering_bias param if needed
            # steering += 0.0

            # CHANGED: clamp steering so the robot can't spin in place
            steering = float(np.clip(steering, -1.0, 1.0))

            speed = self.base_speed * (1.0 - min(0.7, abs(steering)))
            speed = max(0.3, speed)

            twist.linear.x  = float(speed)
            twist.angular.z = float(steering)

            self.get_logger().debug(
                f'cx_near={cx_near} target={target_x} '
                f'err={error_near:+.0f} steer={steering:+.3f}')

        else:
            if (now - self.last_line_time) > self.line_lost_time:
                self.search_mode = True

            if self.search_mode:
                twist.linear.x  = self.search_speed
                twist.angular.z = self.search_turn

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DetectLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    