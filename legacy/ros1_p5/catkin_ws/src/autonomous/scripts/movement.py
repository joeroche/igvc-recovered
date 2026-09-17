#! /usr/bin/env python


import rospy
import sys
from geometry_msgs.msg import Twist
import time
from autonomous.msg import pair
import threading


pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
last_print_time = 0


def callback(data):
    half = data.width // 2
    avg = data.x_avg


    global last_print_time
    current_time = time.time()
    if current_time - last_print_time > 0.5:
        # print("Target Pixel: {}".format(avg))
        # print("Middle Pixel: {}".format(half))
        # print("\n")
        last_print_time = current_time
       
    move = Twist()


    difference_scaling = abs(half-avg)/half
    move.linear.x = 0.9-(difference_scaling*1.5)
    # print("{}".format(difference_scaling))


    #if the midpoint pixel is in left half of screen, rotate the robot right
    if(avg <= half):
        move.angular.z = -2 * (difference_scaling)
    #if the midpoint pixel is in right half of screen, rotate the robot left
    if(avg > half):
        move.angular.z = 2 * (difference_scaling)


    pub.publish(move)


def movement():
    rospy.init_node('movement')
    sub = rospy.Subscriber("/avg_and_width", pair, callback)
    rospy.spin()


movement()
