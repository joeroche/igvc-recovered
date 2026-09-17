#!/usr/bin/env python3

import socket
import threading
import time
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped


def parse_nmea_coord(value: str, hemisphere: str) -> float:
    if not value:
        return 0.0
    raw = float(value)
    degrees = int(raw / 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60.0
    if hemisphere in ['S', 'W']:
        decimal = -decimal
    return decimal


def latlon_to_enu(lat: float, lon: float, alt: float, origin_lat: float, origin_lon: float, origin_alt: float):
    # Approximate local ENU coordinates from lat/lon using a tangent plane.
    earth_radius = 6378137.0
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    mean_lat = math.radians((lat + origin_lat) / 2.0)
    east = d_lon * earth_radius * math.cos(mean_lat)
    north = d_lat * earth_radius
    up = alt - origin_alt
    return east, north, up


class EmlidGpsNode(Node):
    def __init__(self):
        super().__init__('emlid_gps_node')

        self.declare_parameter('emlid_host', '192.168.1.100')
        self.declare_parameter('emlid_port', 2101)
        self.declare_parameter('protocol', 'tcp')
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('publish_local_pose', True)

        self.host = self.get_parameter('emlid_host').value
        self.port = self.get_parameter('emlid_port').value
        self.protocol = self.get_parameter('protocol').value.lower()
        self.frame_id = self.get_parameter('frame_id').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.origin_alt = self.get_parameter('origin_alt').value
        self.publish_local_pose = self.get_parameter('publish_local_pose').value

        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.status_pub = self.create_publisher(NavSatStatus, '/fix_status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/emlid/pose', 10)

        self.get_logger().info(f'Configured Emlid RS+ node for {self.host}:{self.port} over {self.protocol.upper()}')

        self.thread = threading.Thread(target=self.stream_loop, daemon=True)
        self.thread.start()

    def stream_loop(self):
        while rclpy.ok():
            try:
                if self.protocol == 'tcp':
                    self.stream_tcp()
                else:
                    self.stream_udp()
            except Exception as exc:
                self.get_logger().warning(f'Emlid connection error: {exc}')
                time.sleep(2.0)

    def stream_tcp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(5.0)
            self.get_logger().info('Connecting to Emlid RS+ TCP stream...')
            sock.connect((self.host, self.port))
            self.get_logger().info('Connected to Emlid RS+')
            buffer = ''
            while rclpy.ok():
                data = sock.recv(4096).decode('ascii', errors='ignore')
                if not data:
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.handle_nmea(line.strip())

    def stream_udp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            self.get_logger().info('Binding to Emlid RS+ UDP stream...')
            sock.bind((self.host, self.port))
            while rclpy.ok():
                data, _ = sock.recvfrom(4096)
                line = data.decode('ascii', errors='ignore').strip()
                self.handle_nmea(line)

    def handle_nmea(self, sentence: str):
        if not sentence.startswith('$'):
            return

        parts = sentence.split(',')
        sentence_type = parts[0][3:]
        if sentence_type in ['GGA', 'GNGGA', 'GPGGA']:
            self.publish_fix_from_gga(parts)

    def publish_fix_from_gga(self, parts):
        if len(parts) < 10:
            return

        lat = parse_nmea_coord(parts[2], parts[3])
        lon = parse_nmea_coord(parts[4], parts[5])
        quality = int(parts[6]) if parts[6].isdigit() else 0
        hdop = float(parts[8]) if parts[8] else 0.0
        alt = float(parts[9]) if parts[9] else 0.0

        status = NavSatStatus.STATUS_NO_FIX
        if quality >= 1:
            status = NavSatStatus.STATUS_FIX
        if quality >= 4:
            status = NavSatStatus.STATUS_GBAS_FIX

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = self.frame_id
        fix_msg.status.status = status
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = alt
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        covariance = hdop * hdop if hdop > 0 else 100.0
        fix_msg.position_covariance = [covariance, 0.0, 0.0,
                                       0.0, covariance, 0.0,
                                       0.0, 0.0, covariance]

        self.fix_pub.publish(fix_msg)
        self.status_pub.publish(fix_msg.status)

        if self.publish_local_pose:
            x, y, z = latlon_to_enu(lat, lon, alt, self.origin_lat, self.origin_lon, self.origin_alt)
            pose_msg = PoseStamped()
            pose_msg.header = fix_msg.header
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmlidGpsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
