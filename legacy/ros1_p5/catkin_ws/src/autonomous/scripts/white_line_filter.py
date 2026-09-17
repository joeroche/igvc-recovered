#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import struct

def filter_white_lines(input_cloud):
    """
    Filters out all points except white lines from the incoming PointCloud2 data.
    Args:
        input_cloud (sensor_msgs/PointCloud2): Incoming point cloud with x, y, z, and rgb values.
    Returns:
        sensor_msgs/PointCloud2: Filtered point cloud containing only white line points.
    """
    filtered_points = []

    # Read points from the PointCloud2 message
    for point in pc2.read_points(input_cloud, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        x, y, z, rgb_float = point

        # Unpack the packed float (RGB) into integers
        rgb_int = struct.unpack('I', struct.pack('f', rgb_float))[0]  # Convert float to uint32
        r = (rgb_int >> 16) & 0xFF  # Extract red channel
        g = (rgb_int >> 8) & 0xFF   # Extract green channel
        b = rgb_int & 0xFF          # Extract blue channel

        # Keep only white points based on thresholds
        if r > 200 and g > 200 and b > 200:  # Adjust thresholds as needed
            filtered_points.append((x, y, z, rgb_float))  # Keep the white point

    # Create a new PointCloud2 message for the filtered points
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = input_cloud.header.frame_id    
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    filtered_cloud = pc2.create_cloud(header, fields, filtered_points)

    return filtered_cloud

def callback(input_cloud):
    """
    Callback function for processing the incoming PointCloud2 data.
    Args:
        input_cloud: Incoming PointCloud2 message.
    """
    filtered_cloud = filter_white_lines(input_cloud)
    pub.publish(filtered_cloud)

def main():
    rospy.init_node("filter_white_lines_node", anonymous=True)

    # Subscriber to the original PointCloud2 topic
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback, queue_size=10)

    # Publisher for the filtered PointCloud2 topic
    global pub
    pub = rospy.Publisher("filtered_points", PointCloud2, queue_size=10)

    rospy.spin()

if __name__ == "__main__":
    main()
