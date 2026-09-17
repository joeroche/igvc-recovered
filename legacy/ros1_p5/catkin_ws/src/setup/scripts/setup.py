#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import time

def repeat_publisher():
    rospy.init_node('repeat_publisher', anonymous=True)
    pub = rospy.Publisher('vehicle_state', String, queue_size=10, latch=True)
    rate = rospy.Rate(1)  # 1 Hz (publish message every second)
    
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing 'autonomous'")
        pub.publish("a")
        rate.sleep()

if __name__ == '__main__':
    try:
        repeat_publisher()
    except rospy.ROSInterruptException:
        pass
