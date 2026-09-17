#!/usr/bin/env python3
import rospy
import numpy as np
import pyrealsense2 as rs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

def create_pointcloud(depth_image, color_image, intrinsics):
    h, w = depth_image.shape
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy

    xx, yy = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_image.astype(np.float32)
    x = (xx - cx) * z / fx
    y = (yy - cy) * z / fy

    xyz = np.stack((x, y, z), axis=-1).reshape(-1, 3)

    if color_image is not None:
        rgb = color_image.reshape(-1, 3)
        points = np.hstack((xyz, rgb))
    else:
        points = xyz

    return points

def publish_pointcloud(points, pub, frame_id="camera_depth_optical_frame"):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    if points.shape[1] == 6:  # XYZRGB
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("r", 12, PointField.UINT8, 1),
            PointField("g", 13, PointField.UINT8, 1),
            PointField("b", 14, PointField.UINT8, 1),
        ]
    else:  # XYZ only
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]

    cloud_msg = pc2.create_cloud(header, fields, points)
    pub.publish(cloud_msg)

def main():
    rospy.init_node("d435_pointcloud_publisher")

    # Publishers
    pc_pub = rospy.Publisher("/d435/points", PointCloud2, queue_size=1)
    color_pub = rospy.Publisher("/d435/color/image_raw", Image, queue_size=1)
    depth_pub = rospy.Publisher("/d435/depth/image_raw", Image, queue_size=1)

    bridge = CvBridge()

    # RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    rospy.loginfo("Publishing point cloud, depth, and color images...")

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            intr = depth_frame.profile.as_video_stream_profile().get_intrinsics()
            depth_image = np.asanyarray(depth_frame.get_data())  # mm
            color_image = np.asanyarray(color_frame.get_data())  # BGR

            # Publish raw images
            stamp = rospy.Time.now()
            frame_id = "camera_depth_optical_frame"

            color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = frame_id
            color_pub.publish(color_msg)

            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="mono16")  # still in mm
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = frame_id
            depth_pub.publish(depth_msg)

            # Filter invalid points
            mask = depth_image > 0
            depth_in_meters = depth_image.astype(np.float32) * 0.001  # mm → meters

            # Generate and publish point cloud
            xyzrgb = create_pointcloud(depth_in_meters, color_image, intr)
            xyzrgb = xyzrgb[mask.reshape(-1)]
            publish_pointcloud(xyzrgb, pc_pub, frame_id=frame_id)

    finally:
        pipeline.stop()
        rospy.loginfo("Stopped RealSense pipeline.")

if __name__ == "__main__":
    main()
