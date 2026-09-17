import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            '/camera/color/image_raw',  # make sure this topic exists
            self.callback,
            10
        )
        self.frame_count = 0

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print("Conversion error:", e)
            return
        self.frame_count += 1
        print("Frame", self.frame_count)
        cv2.imshow("Color Camera", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraViewer()
    print("Spinning node, waiting for camera frames...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()