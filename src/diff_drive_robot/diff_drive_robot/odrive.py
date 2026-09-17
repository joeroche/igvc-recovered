#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import odrive
from odrive.enums import (
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    InputMode,
    MOTOR_TYPE_HIGH_CURRENT,
)

class ODriveNode(Node):

    def __init__(self):
        super().__init__('odrive_node')

        self.declare_parameter('velocity_scale', 1.0)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('max_angular_rate', 1.2)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('command_timeout_sec', 0.5)

        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_angular_rate = self.get_parameter('max_angular_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.command_timeout = self.get_parameter('command_timeout_sec').value

        self.last_cmd_time = time.time()

        self.get_logger().info('Finding ODrive...')
        self.odrv = odrive.find_any()
        self.odrv.clear_errors()
        self.get_logger().info(f'ODrive connected, vbus_voltage={self.odrv.vbus_voltage}')

        self.configure_odrive()
        self.wait_for_calibration()
        self.enter_closed_loop_control()

        self.vel_pub = self.create_publisher(Twist, 'odrive/motor_velocity', 10)

        self.sub = self.create_subscription(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            self.cmd_callback,
            10
        )

        self.create_timer(0.1, self.watchdog_timer)

    def configure_odrive(self):
        self.get_logger().info('Configuring ODrive parameters...')

        self.odrv.config.dc_max_negative_current = -30

        self.configure_axis(self.odrv.axis0)
        self.configure_axis(self.odrv.axis1)

        self.odrv.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.odrv.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

        if (not self.odrv.axis0.motor.config.pre_calibrated
                or not self.odrv.axis0.encoder.config.pre_calibrated
                or not self.odrv.axis1.motor.config.pre_calibrated
                or not self.odrv.axis1.encoder.config.pre_calibrated):
            self.get_logger().info('ODrive not pre-calibrated, performing full calibration sequence...')
            self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.wait_for_state(self.odrv.axis0, AXIS_STATE_IDLE, timeout=20)
            self.odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.wait_for_state(self.odrv.axis1, AXIS_STATE_IDLE, timeout=20)
            self.get_logger().info('Calibration completed')

    def configure_axis(self, axis):
        axis.motor.config.current_lim = 30
        axis.motor.config.calibration_current = 10
        axis.motor.config.pole_pairs = 3
        axis.motor.config.torque_constant = 0.05
        axis.encoder.config.cpr = 4096
        axis.controller.config.input_filter_bandwidth = 2.0
        axis.controller.config.vel_integrator_gain = 0.16
        axis.controller.config.vel_gain = 0.045
        axis.controller.config.vel_limit = 30
        axis.controller.config.vel_ramp_rate = 75
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.input_mode = InputMode.VEL_RAMP
        axis.motor.config.resistance_calib_max_voltage = 8

    def wait_for_state(self, axis, target_state, timeout=10):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if axis.current_state == target_state:
                return True
            time.sleep(0.1)
        self.get_logger().warning(f'Axis did not reach state {target_state} in {timeout}s')
        return False

    def wait_for_calibration(self):
        if (not self.odrv.axis0.motor.config.pre_calibrated
                or not self.odrv.axis0.encoder.config.pre_calibrated
                or not self.odrv.axis1.motor.config.pre_calibrated
                or not self.odrv.axis1.encoder.config.pre_calibrated):
            self.get_logger().info('Re-running full calibration on both axes...')
            self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.wait_for_state(self.odrv.axis0, AXIS_STATE_IDLE, timeout=20)
            self.odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.wait_for_state(self.odrv.axis1, AXIS_STATE_IDLE, timeout=20)
            self.get_logger().info('Re-calibration completed')

    def enter_closed_loop_control(self):
        self.get_logger().info('Switching ODrive to closed-loop control...')
        self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.wait_for_state(self.odrv.axis0, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout=10)
        self.wait_for_state(self.odrv.axis1, AXIS_STATE_CLOSED_LOOP_CONTROL, timeout=10)
        self.get_logger().info('ODrive is in closed-loop velocity control')

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if linear > 0.05 and abs(angular) > self.max_angular_rate:
            angular = math.copysign(self.max_angular_rate, angular)

        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        left_turns = v_left / (2 * math.pi * self.wheel_radius)
        right_turns = v_right / (2 * math.pi * self.wheel_radius)

        left_turns *= self.velocity_scale
        right_turns *= self.velocity_scale

        left_turns = max(min(left_turns, self.max_velocity), -self.max_velocity)
        right_turns = max(min(right_turns, self.max_velocity), -self.max_velocity)

        self.odrv.axis0.controller.input_vel = left_turns
        self.odrv.axis1.controller.input_vel = right_turns

        velocity_msg = Twist()
        velocity_msg.linear.x = left_turns
        velocity_msg.linear.y = right_turns
        velocity_msg.linear.z = linear
        velocity_msg.angular.z = angular
        self.vel_pub.publish(velocity_msg)

        self.last_cmd_time = time.time()
        self.get_logger().debug(f'cmd_vel received linear={linear:.2f} angular={angular:.2f} -> left={left_turns:.2f} right={right_turns:.2f}')

    def watchdog_timer(self):
        if time.time() - self.last_cmd_time > self.command_timeout:
            self.odrv.axis0.controller.input_vel = 0.0
            self.odrv.axis1.controller.input_vel = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics
        v_left  = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        # Convert m/s → turns/s (ODrive uses turns/sec)
        left_turns  = v_left  / (2 * 3.1415 * self.wheel_radius)
        right_turns = v_right / (2 * 3.1415 * self.wheel_radius)

        # Apply velocity scaling for very slow operation
        left_turns  *= self.velocity_scale
        right_turns *= self.velocity_scale

        # Clamp to maximum velocity to ensure slow operation
        left_turns  = max(min(left_turns, self.max_velocity), -self.max_velocity)
        right_turns = max(min(right_turns, self.max_velocity), -self.max_velocity)

        # Send to motors
        self.odrv.axis0.controller.input_vel = left_turns
        self.odrv.axis1.controller.input_vel = right_turns

        self.get_logger().info(f"L: {left_turns:.2f}, R: {right_turns:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()