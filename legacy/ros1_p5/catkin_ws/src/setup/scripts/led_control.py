#!/usr/bin/env python3

import time
import serial
import glob
import keyboard
import platform
import sys
import rospy
from std_msgs.msg import String

rospy.init_node('led_control')

ser = None

def callback(data):
	global ser
	rospy.loginfo("Recieved vehicle state: " + data.data)

	if data.data == "autonomous":
		ser.write(b'a') # just a character for the arduino to recieve 
		rospy.loginfo("Writing a: " + data.data)

	else:
		rospy.loginfo("Writing b: " + data.data)
		ser.write(b'b')


def listener():
	global ser
	if(platform.system() != "Linux"):
		sys.exit("Linux only")

	while(True):
		probable_port = glob.glob("/dev/ttyUSB0")
		if len(probable_port) > 0:
			ser = serial.Serial(probable_port[0], 9600, timeout = 5)
			#ser.write('a'.encode('utf-8')) # just a character for the arduino to recieve 
			rospy.loginfo("Connected")
			break
		time.sleep(1)

	rospy.loginfo("Startng listener")
	rospy.Subscriber("vehicle_state", String, callback)
	rospy.spin()
# if __name__ == '__main__':
listener()


# testing

	# while(True):
	# 	key = keyboard.read_key()
	# 	print("key pressed:", end="")
	# 	print(key)

	# 	ser.write(key.encode('utf-8'))
