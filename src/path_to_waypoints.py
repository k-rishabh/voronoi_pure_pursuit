#!/usr/bin/python3

import rospy
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64MultiArray 

path = Path()
def get_path(data):
    global path
    path = data

def main():
    rospy.init_node("path_to_waypoints")
    rospy.sleep(1)
    
    waypoints_pub = rospy.Publisher("/waypoints", Float64MultiArray, queue_size=1)
    rospy.sleep(1)
    
    p_waypoints = Float64MultiArray()

    global path
    rospy.Subscriber("/test_rob/path_unsynced", Path, get_path)
    flag = 0
    while not rospy.is_shutdown():
        n = len(path.poses)
        waypoints = []
        for i in range(0, n):
            curr_position = path.poses[i].pose.position
            orientation_q = path.poses[i].pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            global roll, pitch, yaw
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            yaw = yaw*57.296 #convert radians to degrees
            waypoints.append([curr_position.x, curr_position.y, yaw])
        
        print(waypoints)
        # p_waypoints.data = waypoints
        # waypoints_pub.publish(p_waypoints)
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()