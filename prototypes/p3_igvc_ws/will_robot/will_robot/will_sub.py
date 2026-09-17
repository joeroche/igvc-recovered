import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np


class WillSub(Node):
    def __init__(self):
        super().__init__("will_sub")
        self.subscription_rgb=self.create_subscription(Image, "/camera/realsense/color/image_raw",self.rgb_frame_callback,100)
        self.will_pub_mask=self.create_publisher(Image,"mask",10)
        self.br_rgb = CvBridge()

    def rgb_frame_callback(self,data):
        self.get_logger().warning("Recieving")
        current_frame = self.br_rgb.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        lower_tan = np.array([10, 50,50])
        upper_tan = np.array([50, 160, 255])        

        # lower_red2 = np.array([170, 70, 50])
        # upper_red2 = np.array([180, 255, 255])        

        # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # mask = cv2.bitwise_or(mask1, mask2)




        mask= cv2.inRange(hsv,lower_tan,upper_tan)

        self.will_pub_mask.publish(self.br_rgb.cv2_to_imgmsg(mask))
        cv2.imshow("RGB",mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    will_sub = WillSub()
    rclpy.spin(will_sub)
    will_sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()