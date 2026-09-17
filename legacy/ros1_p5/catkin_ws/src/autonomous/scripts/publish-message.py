#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import signal
import sys

def publish_kill_message(pub):
    rospy.loginfo("Publishing 'kill'")
    pub.publish("kill")

def signal_handler(sig, frame):
    # Ensure the publisher is latched so subscribers get the message even if this node exits immediately after
    pub = rospy.Publisher('vehicle_state', String, queue_size=10, latch=True)
    rospy.sleep(1)  # Ensure the publisher connects
    publish_kill_message(pub)
    rospy.signal_shutdown('Shutdown signal received')

def repeat_publisher():
    rospy.init_node('repeat_publisher', anonymous=True)
    pub = rospy.Publisher('vehicle_state', String, queue_size=10, latch=True)
    rate = rospy.Rate(1)  # 1 Hz (publish message every second)

    # Register signal handler to publish "kill" on shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    while not rospy.is_shutdown():
        rospy.loginfo("Publishing 'autonomous'")
        pub.publish("autonomous")
        rate.sleep()

if __name__ == '__main__':
    try:
        repeat_publisher()
    except rospy.ROSInterruptException:
        pass
