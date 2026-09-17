#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import struct

class WhiteFilter(Node):
    def __init__(self):
        super().__init__('white_filter')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(PointCloud2, '/oak/points', self.callback, qos)
        self.pub = self.create_publisher(PointCloud2, '/oak/points_white', qos)


    def callback(self, msg):
        points = pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        filtered_points = []

        white_thresh = 150  
        for p in points:
            x, y, z, rgb = p
            rgb_bytes = struct.pack('f', rgb)
            r, g, b, _ = struct.unpack('BBBB', rgb_bytes)
            if r > white_thresh and g > white_thresh and b > white_thresh:
                filtered_points.append([x, y, z, rgb])

        if not filtered_points:
            return

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = pc2.create_cloud(msg.header, fields, filtered_points)
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = WhiteFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
