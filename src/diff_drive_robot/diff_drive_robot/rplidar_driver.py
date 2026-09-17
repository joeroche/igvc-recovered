#!/usr/bin/env python3

"""
RPLidar A1 ROS2 Driver Node
----------------------------
Publishes sensor_msgs/LaserScan on /scan using the rplidar-robotics library.

Dependencies:
    pip install rplidar-robotics
   
ROS2 package dependencies (package.xml):
    rclpy, sensor_msgs, std_msgs

Usage:
    ros2 run <your_package> rplidar_a1_driver
    ros2 run <your_package> rplidar_a1_driver --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rplidar import RPLidar, RPLidarException


class RPLidarA1Node(Node):
    """
    ROS2 node that interfaces with the RPLidar A1 via the rplidar-robotics
    library and publishes LaserScan messages on the /scan topic.
    """

    # A1 hardware constants
    ANGLE_MIN_DEG  = 0.0
    ANGLE_MAX_DEG  = 359.0
    RANGE_MIN_M    = 0.15   # 15 cm minimum reliable range
    RANGE_MAX_M    = 12.0   # 12 m maximum range for A1

    def __init__(self):
        super().__init__('rplidar_a1_driver')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('port',       '/dev/ttyACM0')
        self.declare_parameter('baudrate',   115200)
        self.declare_parameter('frame_id',   'laser')
        self.declare_parameter('topic',      '/scan')
        self.declare_parameter('scan_mode',  '')          # '' = default mode
        self.declare_parameter('inverted',   False)       # flip angle direction
        self.declare_parameter('angle_compensate', True)  # interpolate gaps

        self.port      = self.get_parameter('port').value
        self.baudrate  = self.get_parameter('baudrate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.topic     = self.get_parameter('topic').value
        self.inverted  = self.get_parameter('inverted').value
        self.angle_compensate = self.get_parameter('angle_compensate').value

        # ── QoS – sensor data profile ────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(LaserScan, self.topic, qos)
        self.get_logger().info(f'Publishing LaserScan on "{self.topic}"')

        # ── Connect to hardware ──────────────────────────────────────────────
        self.lidar = None
        self._connect()

        # ── Spin in a timer so ROS can still process signals ─────────────────
        # The actual scan loop is blocking, so we run it in a one-shot timer
        # that reschedules itself after each scan to remain responsive.
        self._scan_iter  = None   # holds the active iter_scans() iterator
        self._scan_timer = self.create_timer(0.0, self._scan_once)

    # ────────────────────────────────────────────────────────────────────────
    # Hardware helpers
    # ────────────────────────────────────────────────────────────────────────

    def _connect(self):
        """Open serial connection and start the motor."""
        try:
            self.get_logger().info(
                f'Connecting to RPLidar A1 on {self.port} @ {self.baudrate} baud …'
            )

            # ── Step 1: flush OS serial buffer BEFORE library opens port ────
            # Stale bytes left by a previous crashed session corrupt the very
            # first descriptor read. We open the port raw with pyserial,
            # drain it, then close so RPLidar() can reopen cleanly.
            import serial as _serial
            with _serial.Serial(self.port, baudrate=self.baudrate, timeout=0.5) as _pre:
                _pre.reset_input_buffer()
                _pre.reset_output_buffer()
            time.sleep(0.2)

            # ── Step 2: open via RPLidar library ─────────────────────────────
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)

            # ── Step 3: send stop commands and let the device settle ─────────
            # stop() sends the STOP byte; stop_motor() cuts PWM.
            # Two rounds ensure any partially-sent packet is flushed out.
            self.lidar.stop()
            time.sleep(0.2)
            self.lidar.stop_motor()
            time.sleep(0.5)

            # ── Step 4: drain anything the device sent back ──────────────────
            # Walk all known attribute names for the underlying serial object.
            for _attr in ('_serial', 'serial', '_port', 'port'):
                _ser = getattr(self.lidar, _attr, None)
                if _ser is not None and hasattr(_ser, 'reset_input_buffer'):
                    _ser.reset_input_buffer()
                    self.get_logger().info(f'Buffer flushed via lidar.{_attr}')
                    break
            else:
                self.get_logger().warn(
                    'Could not find serial attribute — skipping post-stop flush.'
                )

            # ── Step 5: start motor and wait for full speed ──────────────────
            self.lidar.start_motor()
            self.get_logger().info('Motor started — warming up (2 s) …')
            time.sleep(2.0)

            # ── Step 6: read device info now that state is clean ─────────────
            # get_info() / get_health() are informational only; log but don't
            # crash if they fail — the scan loop will surface real errors.
            try:
                info   = self.lidar.get_info()
                health = self.lidar.get_health()
                self.get_logger().info(f'Device info   : {info}')
                self.get_logger().info(f'Device health : {health}')
                if health[0] == 'Warning':
                    self.get_logger().warn('RPLidar reports WARNING health status.')
                elif health[0] == 'Error':
                    self.get_logger().error(f'RPLidar health error: {health}')
            except RPLidarException as exc:
                self.get_logger().warn(
                    f'Could not read device info (non-fatal): {exc}'
                )

        except Exception as exc:
            self.get_logger().error(f'Failed to connect to RPLidar: {exc}')
            self._shutdown()
            raise

    def _disconnect(self):
        """Stop motor and close serial port cleanly."""
        if self.lidar is not None:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                self.get_logger().info('RPLidar disconnected.')
            except Exception as exc:
                self.get_logger().warn(f'Error during disconnect: {exc}')
            finally:
                self.lidar = None

    def _shutdown(self):
        self._disconnect()
        if rclpy.ok():
            rclpy.shutdown()

    # ────────────────────────────────────────────────────────────────────────
    # Scan loop (called once per timer tick)
    # ────────────────────────────────────────────────────────────────────────

    def _scan_once(self):
        """
        Collect one full 360° scan from the iterator, build a LaserScan
        message, and publish it. Reschedules itself immediately after.

        Bad frames (descriptor length mismatch, etc.) are caught and the
        iterator is recreated to resync with the sensor instead of crashing.
        """
        # Cancel timer so it doesn't pile up while we are in a blocking call.
        self._scan_timer.cancel()

        try:
            # Create the iterator on first call (or after a resync)
            if self._scan_iter is None:
                self._scan_iter = self.lidar.iter_scans(max_buf_meas=500)

            if not rclpy.ok():
                return

            try:
                # Advance one full rotation
                scan = next(self._scan_iter)
                msg  = self._build_laserscan(scan)
                if msg is not None:
                    self.publisher_.publish(msg)

            except RPLidarException as exc:
                # Bad frame — log, discard iterator, resync on next tick
                self.get_logger().warn(
                    f'Bad scan frame (resyncing): {exc}'
                )
                self._scan_iter = None   # will be recreated next tick

            except StopIteration:
                self.get_logger().warn('Scan iterator exhausted — resyncing.')
                self._scan_iter = None

        except Exception as exc:
            self.get_logger().error(f'Unexpected error in scan loop: {exc}')
            self._scan_iter = None
        finally:
            # Reschedule for the next spin cycle
            if rclpy.ok():
                self._scan_timer = self.create_timer(0.0, self._scan_once)

    # ────────────────────────────────────────────────────────────────────────
    # Message builder
    # ────────────────────────────────────────────────────────────────────────

    def _build_laserscan(self, raw_scan) -> LaserScan | None:
        """
        Convert raw RPLidar measurements into a sensor_msgs/LaserScan.

        raw_scan  : list of (quality, angle_deg, distance_mm)
        returns   : LaserScan message or None if scan is empty
        """
        if not raw_scan:
            return None

        # ── Angular resolution ───────────────────────────────────────────────
        # A1 nominally does ~360 samples/rev at ~5.5 Hz → ~1°/sample.
        # We build a fixed-resolution array and fill in measured points.
        NUM_BINS   = 360
        angle_step = 2.0 * math.pi / NUM_BINS   # radians per bin

        ranges      = [float('inf')] * NUM_BINS
        intensities = [0.0]          * NUM_BINS

        for quality, angle_deg, distance_mm in raw_scan:
            if quality == 0:
                # Low-quality / invalid measurement – skip
                continue

            distance_m = distance_mm / 1000.0
    
            # Clamp to sensor limits
            if not (self.RANGE_MIN_M <= distance_m <= self.RANGE_MAX_M):
                continue

            # Optionally invert rotation direction
            if self.inverted:
                angle_deg = 360.0 - angle_deg

            # Wrap angle into [0, 360)
            angle_deg = angle_deg % 360.0
            bin_idx   = int(angle_deg) % NUM_BINS

            # Keep the closest reading if multiple hits fall in the same bin
            if distance_m < ranges[bin_idx]:
                ranges[bin_idx]      = distance_m
                intensities[bin_idx] = float(quality)

        # ── Optional gap interpolation ────────────────────────────────────────
        if self.angle_compensate:
            ranges      = self._interpolate_gaps(ranges)

        # ── Populate LaserScan ────────────────────────────────────────────────
        msg = LaserScan()

        msg.header          = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = math.radians(self.ANGLE_MIN_DEG)
        msg.angle_max       = math.radians(self.ANGLE_MAX_DEG)
        msg.angle_increment = angle_step
        msg.time_increment  = 0.0          # unknown without encoder feedback
        msg.scan_time       = 1.0 / 5.5   # ~5.5 Hz nominal for A1
        msg.range_min       = self.RANGE_MIN_M
        msg.range_max       = self.RANGE_MAX_M

        msg.ranges          = ranges
        msg.intensities     = intensities

        return msg

    @staticmethod
    def _interpolate_gaps(ranges: list) -> list:
        """
        Linear interpolation to fill inf bins that are flanked by valid
        readings on both sides (gap ≤ 3 bins). Reduces NaN/inf noise in
        downstream SLAM/navigation stacks.
        """
        n      = len(ranges)
        result = ranges[:]

        i = 0
        while i < n:
            if math.isinf(result[i]):
                # Find the start and end of the gap
                gap_start = i
                while i < n and math.isinf(result[i]):
                    i += 1
                gap_end = i  # exclusive

                gap_len = gap_end - gap_start
                if gap_len <= 3:
                    left  = result[(gap_start - 1) % n]
                    right = result[gap_end % n]
                    if not math.isinf(left) and not math.isinf(right):
                        for j in range(gap_len):
                            t = (j + 1) / (gap_len + 1)
                            result[(gap_start + j) % n] = left + t * (right - left)
            else:
                i += 1

        return result

    # ────────────────────────────────────────────────────────────────────────
    # Node lifecycle
    # ────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Called by rclpy on shutdown – ensure hardware is released."""
        self.get_logger().info('Shutting down RPLidar node …')
        self._disconnect()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarA1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()