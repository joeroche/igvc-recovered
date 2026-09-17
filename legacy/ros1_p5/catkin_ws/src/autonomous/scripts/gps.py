#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import NavSatFix
import math


# Waypoint (lat, lon) in decimal degrees
WAYPOINT_LAT = 40.521723
WAYPOINT_LON = -74.46044


def haversine(lat1, lon1, lat2, lon2):
   R = 6371000  # Earth radius in meters
   phi1 = math.radians(lat1)
   phi2 = math.radians(lat2)
   dphi = math.radians(lat2 - lat1)
   dlambda = math.radians(lon2 - lon1)


   a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
   c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


   return R * c  # in meters


def gps_callback(msg):
   current_lat = msg.latitude
   current_lon = msg.longitude


   distance = haversine(current_lat, current_lon, WAYPOINT_LAT, WAYPOINT_LON)
   rospy.loginfo(f"Distance to waypoint: {distance:.2f} meters")


def main():
   rospy.init_node('gps_waypoint_distance')
   rospy.Subscriber('/fix', NavSatFix, gps_callback)
   rospy.loginfo("Waiting for GPS data...")
   rospy.spin()


if __name__ == '__main__':
   main()




