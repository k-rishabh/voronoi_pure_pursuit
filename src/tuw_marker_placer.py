#!/usr/bin/python
# license removed for brevity

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tuw_multi_robot_msgs.msg import Graph
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String

pub = rospy.Publisher('tuw_lines', Marker, queue_size=10)


def callback(data):
    global pub
    #rospy.loginfo(rospy.get_caller_id() + " I heard something!")
    res=0.032
    #rospy.loginfo(rospy.get_caller_id() + " I heard %f ",res)
    orig=data.origin
    pub_array=Marker()
    pub_array.header=data.header
    pub_array.header.stamp=rospy.get_rostime()
    pub_array.type=pub_array.LINE_LIST #5
    pub_array.action = pub_array.ADD
    pub_array.lifetime = rospy.Duration()
    pub_array.id=1
    pub_array.ns="Voronoi"
    pub_array.scale.x = 0.05
    color=ColorRGBA()
    color.r=1.0
    color.a=1.0   
    pub_array.color=color
    pub_array.pose.position.x = 0.0
    pub_array.pose.position.y = 0.0
    pub_array.pose.position.z = 0.0
    pub_array.pose.orientation.w = 1.0
    
    for i in range(0,len(data.vertices)):
        point_pose=Point()
        point_pose.x=(data.vertices[i].path[0].x * res)+orig.position.x #first point in vertex
        point_pose.y=(data.vertices[i].path[0].y * res)+orig.position.y
        pub_array.points.append(point_pose)
        pub_array.colors.append(color)
        point_pose=Point()
        point_pose.x=(data.vertices[i].path[data.vertices[i].weight-1].x * res)+orig.position.x  #last point in vertex
        point_pose.y=(data.vertices[i].path[data.vertices[i].weight-1].y * res)+orig.position.y
        #point_pose.z=1.0
        pub_array.points.append(point_pose)
        pub_array.colors.append(color)
    # rospy.loginfo(rospy.get_caller_id() + " P1 %d %f %f ",len(pub_array.points),pub_array.points[0].x,pub_array.points[0].y)
    # rospy.loginfo(rospy.get_caller_id() + " P2 %d %f %f ",len(pub_array.points),pub_array.points[1].x,pub_array.points[1].y)
    pub.publish(pub_array)


def talker():
    rospy.init_node('tuw_marker_placer', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/segments", Graph, callback)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
