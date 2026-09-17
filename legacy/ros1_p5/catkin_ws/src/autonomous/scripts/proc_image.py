#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('autonomous')

import sys
import rospy
import numpy as np
import cv2
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from autonomous.msg import pair
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge, CvBridgeError
from heapq import heapify, heappush, heappop

WAYPOINT_LAT = 42.6682223
WAYPOINT_LON = -83.217802


def draw_keypoints(img, keypoints, color):
    for kp in keypoints:
        x, y = kp.pt
        cv2.circle(img, (int(x), int(y)), color=color, radius=10, thickness=-1)


class image_converter:
  
    def gps_callback(self, msg):
        current_lat = msg.latitude
        current_lon = msg.longitude

        def haversine(lat1, lon1, lat2, lon2):
                R = 6371000  # Earth radius in meters
                phi1 = math.radians(lat1)
                phi2 = math.radians(lat2)
                dphi = math.radians(lat2 - lat1)
                dlambda = math.radians(lon2 - lon1)

                a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

                return R * c  # in meters

        self.distance = haversine(current_lat, current_lon, WAYPOINT_LAT, WAYPOINT_LON)
        rospy.loginfo(f"Distance to waypoint: {self.distance:.2f} meters")
            


    def __init__(self):
        self.image_pub = rospy.Publisher("/image_raw", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=10)
        self.x_avg_pub = rospy.Publisher("/avg_and_width", pair, queue_size=10)
        self.move_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gps_callback)
        self.distance = 1000

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # IPM
        src_points = np.float32([[273, 142], [360, 142], [510, 364], [123, 364]])
        dst_points = np.float32([[200, 0], [440, 0], [440, 480], [200, 480]])
        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped = cv2.warpPerspective(cv_image, M, (cv_image.shape[1], cv_image.shape[0]))
        (h, w) = warped.shape[:2]

        scale = 233
        widthScale = 0
        heap = []
        heapify(heap)

        cropped_image = warped[(scale):(h - 100), widthScale:(w - widthScale)]
        hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (10, 200, 50), (20, 255, 220))
        inverted_img = cv2.bitwise_not(mask)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = False
        params.minArea = 10
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(inverted_img)

        for point in keypoints:
            heappush(heap, [point.pt[0], point.pt[1]])

        for point in keypoints:
            point.pt = (point.pt[0] + widthScale, point.pt[1] + scale)

        draw_keypoints(warped, keypoints, (20, 255, 100))
        draw_keypoints(hsv, keypoints, (20, 255, 100))

        grayImage = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(grayImage, (7, 7), 0)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        sensitivity = 15
        mask = cv2.inRange(hsv, (0, 0, 255 - sensitivity), (255, sensitivity, 255))
        inverted_img = cv2.bitwise_not(mask)
        v = np.median(blur)
        sigma = 0.33
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edge = cv2.Canny(inverted_img, lower + 3000, upper + 3000, apertureSize=5)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(edge, kernel, iterations=1)

        cropped_image_lines = dilation[scale:(h - 100), widthScale:(w - widthScale)]
        cv2.waitKey(1)

        lines = cv2.HoughLinesP(cropped_image_lines, 1, np.pi / 180, 20, minLineLength=150, maxLineGap=200)

        x1_left = h
        x1_right = 0
        x2_left = h
        x2_right = 0
        y1_left = h
        y1_right = 0
        y2_left = h
        y2_right = 0
        
        if self.distance < 5:
            rospy.signal_shutdown("reached coordinate")

        if lines is None or len(lines) == 1:
            print("LINES NOT DETECTED, MOVING BACKWARDS\n")
            move = Twist()
            move.linear.x = -1
            # self.move_pub.publish(move)
        else:
            for cur in lines:
                x1, y1, x2, y2 = cur[0]
                if x1 < x1_left and x2 < x2_left:
                    y1_left, y2_left = y1, y2
                    x1_left, x2_left = x1, x2
                elif x1 > x1_right and x2 > x2_right:
                    y1_right, y2_right = y1, y2
                    x1_right, x2_right = x1, x2

            cv2.line(warped, (x1_right + widthScale, y1_right + scale),
                     (x2_right + widthScale, y2_right + scale), (0, 255, 0), 12)
            cv2.line(warped, (x1_left + widthScale, y1_left + scale),
                     (x2_left + widthScale, y2_left + scale), (0, 255, 0), 12)

            left_avg_x = (x1_left + x2_left) / 2
            right_avg_x = (x1_right + x2_right) / 2
            left_avg_y = (y1_left + y2_left) / 2
            right_avg_y = (y1_right + y2_right) / 2
            heappush(heap, [left_avg_x, left_avg_y])
            heappush(heap, [right_avg_x, right_avg_y])

            avg_x1_x2 = 0
            biggest_gap = 0
            (h_cropped, w_cropped) = cropped_image_lines.shape[:2]
            while len(heap) != 1:
                removed = heappop(heap)
                current = heap[0]
                difference = current[0] - removed[0]
                if difference > biggest_gap:
                    biggest_gap = difference
                    left = removed
                    right = current
                    avg_x1_x2 = int(left[0] + (difference / 2))

            cone_shift_scale = 100
            if (left[0] == left_avg_x and right[0] != right_avg_x) or left[0] == right_avg_x:
                avg_x1_x2 -= int(cone_shift_scale * (right[1] / h_cropped))
            elif (left[0] != left_avg_x and right[0] == right_avg_x) or right[0] == left_avg_x:
                avg_x1_x2 += int(cone_shift_scale * (left[1] / h_cropped))

            cv2.circle(warped, (avg_x1_x2 + widthScale, h // 2), 10, (255, 0, 0), thickness=-1)

            pubPair = pair()
            pubPair.x_avg = avg_x1_x2 + widthScale
            if pubPair.x_avg < 0:
                pubPair.x_avg = 320
            pubPair.width = w
            self.x_avg_pub.publish(pubPair)


def main(args):
    rospy.init_node('image_converter')
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
