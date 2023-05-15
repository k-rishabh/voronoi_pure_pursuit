#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import MarkerArray
from rail_manipulation_msgs.msg import SegmentedObjectList
from rail_manipulation_msgs.msg import SegmentedObject
# Global Variables
object_arr = SegmentedObjectList()

# 2D array to Occupancy Grid
def matrixToArray(height, width, ob_map):
    arr_map = [-1]*(width*height)
    i = width*height        

    for a in range(0, height) :
        for b in range(0, width) :
            i -= 1
            arr_map[i] = int(ob_map[a][b])
    return arr_map

class Grid:
    old_map = OccupancyGrid()
    ready = 0
    
    def __init__(self):
        rospy.Subscriber("/map", OccupancyGrid, self.get_map)

    def get_map(self, data):
        self.old_map = data
        self.ready = 1

    def add_obstacle_square(self, obstacle, curr_map):
        print("In add_obstacle square")

        width = curr_map.info.width
        height = curr_map.info.height

        obs_position_x = obstacle.bounding_volume.pose.pose.position.x
        obs_position_y = obstacle.bounding_volume.pose.pose.position.y
       
        # o_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]
        x_coord = int((obs_position_x - curr_map.info.origin.position.x)/curr_map.info.resolution)
        y_coord = int((obs_position_y - curr_map.info.origin.position.y)/curr_map.info.resolution)
       

        x_scale = int((obstacle.bounding_volume.dimensions.x / curr_map.info.resolution)/2)
        y_scale = int((obstacle.bounding_volume.dimensions.y / curr_map.info.resolution)/2)

        #initialize occupancy grid array
        obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]

        for i in range(0, height):
            for j in range(0, width):
                if (width-j-1)>(x_coord - x_scale) and (width-j-1)<(x_coord+x_scale) and (height-i-1)>(y_coord - y_scale) and (height-i-1)<(y_coord+y_scale): 
                    obstacle_map[i][j] = 100
                else:
                    obstacle_map[i][j] = curr_map.data[(height*width-1)-(j+(i*width))]
        
        mod_map = curr_map
        mod_map.data = matrixToArray(height,width,obstacle_map)
        
        return mod_map
    
    
    def add_obstacle(self, obstacle, curr_map):
        return self.add_obstacle_square(obstacle, curr_map)
    


def get_object_arr(data):
    global get_object_arr
    object_arr = data

def main():
    rospy.init_node("voronoi_add_obstacle")
    global object_arr
    grid = Grid()
    rospy.sleep(1)
    curr_map = grid.old_map
    
    map_pub = rospy.Publisher("/map_edit", OccupancyGrid, queue_size=1)
    rospy.sleep(1)

    # # rospy.Subscriber("/obstacle_marker_arr", MarkerArray, get_marker_arr)
    # rospy.Subscriber("/rail_segmentation/segmented_objects", SegmentedObjectList, get_object_arr)
    dummy_obs = SegmentedObject()
    dummy_obs.bounding_volume.pose.pose.position.x = 0
    dummy_obs.bounding_volume.pose.pose.position.y = 0
    dummy_obs.bounding_volume.dimensions.x = 2
    dummy_obs.bounding_volume.dimensions.y = 2

    object_arr.objects.append(dummy_obs)

    while not rospy.is_shutdown():
        new_map = OccupancyGrid()
        if(grid.ready):
            for obstacle in object_arr.objects:
                new_map = grid.add_obstacle(obstacle, curr_map)
                curr_map = new_map
            map_pub.publish(curr_map)
        rospy.sleep(0.1)    

if __name__ == '__main__':
    main()
