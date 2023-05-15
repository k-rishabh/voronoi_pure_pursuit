#!/usr/bin/python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point,PoseWithCovariance,Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tuw_multi_robot_msgs.msg import Graph,RobotInfo
from nav_msgs.msg import Path

from std_msgs.msg import ColorRGBA
from std_msgs.msg import String

pub = rospy.Publisher('tuw_robot_path', Marker, queue_size=10)
recd_pose=Point()

def saveStartLoc(data):
    recd_pose.x=data.x
    recd_pose.y=data.y

def drawPath(data):
    global pub
    rospy.loginfo(rospy.get_caller_id() + " I heard something!")
    res=0.032
    #rospy.loginfo(rospy.get_caller_id() + " I heard %f ",res)
    orig=Pose()
    pub_array=Marker()
    pub_array.header=data.header
    pub_array.header.stamp=rospy.get_rostime()
    pub_array.type=pub_array.LINE_LIST #5
    pub_array.action = pub_array.ADD
    pub_array.lifetime = rospy.Duration()
    pub_array.id=1
    pub_array.ns="Path"
    pub_array.scale.x = 0.07
    color=ColorRGBA()
    color.g=1.0     #green is path
    color.a=1.0   
    pub_array.color=color
    pub_array.pose.position.x = 0.0
    pub_array.pose.position.y = 0.0
    pub_array.pose.position.z = 0.0
    pub_array.pose.orientation.w = 1.0
    
    for i in range(0,len(data.poses)-1):
        point_pose=Point()
        point_pose.x=data.poses[i].pose.position.x #first point in vertex
        point_pose.y=data.poses[i].pose.position.y
        point_pose.z=0.1
        pub_array.points.append(point_pose)
        pub_array.colors.append(color)
        point_pose=Point()
        point_pose.x=data.poses[i+1].pose.position.x  #last point in vertex
        point_pose.y=data.poses[i+1].pose.position.y
        point_pose.z=0.1
        pub_array.points.append(point_pose)
        pub_array.colors.append(color)
    #rospy.loginfo(rospy.get_caller_id() + " P1 %d %f %f ",len(pub_array.points),pub_array.points[0].x,pub_array.points[0].y)
    #rospy.loginfo(rospy.get_caller_id() + " P2 %d %f %f ",len(pub_array.points),pub_array.points[1].x,pub_array.points[1].y)
    pub.publish(pub_array)    

def rob_status():
    global recd_pose
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/start_loc", Point, saveStartLoc)
    rospy.Subscriber("/test_rob/path_unsynced", Path, drawPath)
    pub_const=rospy.Publisher('/robot_info',RobotInfo, queue_size=10)
    pub_robotInfo=RobotInfo()
    pub_robotInfo.robot_name='test_rob'
    pub_robotInfo.header.frame_id='map'
    pub_robotInfo.header.stamp=rospy.get_rostime()
    pub_robotInfo.mode=1   #idle
    pub_robotInfo.status=2  #completed last job
    posecovar=PoseWithCovariance()
    pub_robotInfo.pose=posecovar
    pub_robotInfo.pose.pose.position.x=recd_pose.x
    pub_robotInfo.pose.pose.position.x=recd_pose.y
    pub_robotInfo.shape_variables.append(0.2)
    while not rospy.is_shutdown():
        pub_const.publish(pub_robotInfo)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('tuw_robot_nav', anonymous=True)
        rob_status()    #continuously publish robot state
    except rospy.ROSInterruptException:
        pass
