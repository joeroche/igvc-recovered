import rospy
from std_msgs.msg import Header, Float64
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import tf
import utm
from math import radians
from tf.transformations import quaternion_from_euler

listener = None

def send_move_base_goal(posx, posy, quat, id):
    global listener
    pub_goal = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1, latch=True)
    
    ag = MoveBaseActionGoal()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = posx
    pose.pose.position.y = posy
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    map_pose = listener.transformPose("map", pose)
    
    ag.goal = MoveBaseGoal()   
    ag.header = Header()
    ag.header.stamp = rospy.Time.now()
    ag.header.frame_id = "map"
    ag.goal_id = GoalID()
    ag.goal_id.id = str(id)
    ag.goal_id.stamp = ag.header.stamp
    ag.goal.target_pose.pose.position.x = map_pose.pose.position.x
    ag.goal.target_pose.pose.position.y = map_pose.pose.position.y
    ag.goal.target_pose.header = ag.header
    ag.goal.target_pose.pose.orientation.x = map_pose.pose.orientation.x
    ag.goal.target_pose.pose.orientation.y = map_pose.pose.orientation.y
    ag.goal.target_pose.pose.orientation.z = map_pose.pose.orientation.z
    ag.goal.target_pose.pose.orientation.w = map_pose.pose.orientation.w
    
    pub_goal.publish(ag)

def gps_callback(data):
    global x, y
    (x, y, _, _) = utm.from_latlon(data.latitude, data.longitude)
    print(f"GPS Received: {x}, {y}")

if __name__ == "__main__":
    rospy.init_node("robotcontroller")
    listener = tf.TransformListener()

    rospy.Subscriber("/tcpfix", NavSatFix, gps_callback)
    rospy.sleep(5)

    f = open("/home/rieee/catkin_ws/src/autonomous/scripts/points.txt")
    lines = f.readlines()
    id = 0
    
    for l in lines:
        print(l)
        m = l.split()
        lat = float(m[0])
        lon = float(m[1])
        quat = [float(n) for n in m[2:]]
        
        # Convert lat/lon to UTM
        posx, posy, _, _ = utm.from_latlon(lat, lon)
        
        send_move_base_goal(posx, posy, quat, id)

        print(f"Coordinates: ({posx}, {posy}) Quaternion: {quat}")
        
        checker = True
        while checker:
            statusList = rospy.wait_for_message("move_base/status", GoalStatusArray)
            for g in statusList.status_list:
                if g.goal_id.id == str(id) and g.status == 3:
                    checker = False
                    break
            rospy.sleep(1)

        id += 1 

    rospy.spin()

    # Publishes GPS fix, velocity, and time reference
    # /tcpfix - NavSatFix
    # /tcpvel - TwistedStamped
    # /tcptime - TimeReference
