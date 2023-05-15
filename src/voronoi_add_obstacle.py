#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid
# from visualization_msgs.msg import MarkerArray
from rail_manipulation_msgs.msg import SegmentedObjectList

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
        rospy.Subscriber("/map", OccupancyGrid, self.get_map) #subscriber to initial map

    def get_map(self, data):
        # Initialize the old map variable with subscriber data
        self.old_map = data 
        self.ready = 1

    def add_obstacle(self, obstacle, curr_map):

        width = curr_map.info.width #width of the map
        height = curr_map.info.height   #height of the map

        obs_position_x = obstacle.bounding_volume.pose.pose.position.x #obstacle position x
        obs_position_y = obstacle.bounding_volume.pose.pose.position.y #obstacle position y
        # o_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]

        # x and y coordinates in terms of pixels
        x_coord = int(obs_position_x - curr_map.info.origin.position.x/curr_map.info.resolution)
        y_coord = int(obs_position_y - curr_map.info.origin.position.y/curr_map.info.resolution)

        # distance of boundaries of the obstacle from the centre
        x_scale = int((obstacle.bounding_volume.dimensions.x / curr_map.info.resolution)/2)
        y_scale = int((obstacle.bounding_volume.dimensions.y / curr_map.info.resolution)/2)

        #initialize occupancy grid array
        obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]

        for i in range(0, height):
            for j in range(0, width):
                curr_position_x = width-j-1
                curr_position_y = height-i-1

                # condition to check if the current pixel lies in the boundaries of the object
                if curr_position_x>(x_coord - x_scale) and curr_position_x<(x_coord+x_scale) and curr_position_y>(y_coord - y_scale) and curr_position_y<(y_coord+y_scale): 
                    obstacle_map[i][j] = 100
                else:
                    #obstacle_map[i][j] = self.curr_map.data[(height*width-1)-(i+(j*height))]
                    obstacle_map[i][j] = curr_map.data[(height*width-1)-(j+(i*width))]
        
        mod_map = curr_map
        # convert 2d array to occupancy grid
        mod_map.data = matrixToArray(height,width,obstacle_map)
        
        #return the modified map with the obstacle
        return mod_map


#for initializing the object arr with subscribed data
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
    rospy.Subscriber("/rail_segmentation/segmented_objects", SegmentedObjectList, get_object_arr)

    while not rospy.is_shutdown():
        new_map = OccupancyGrid()
        if(grid.ready):
            # add every obstacle in array one by one
            for obstacle in object_arr.objects:
                new_map = grid.add_obstacle(obstacle, curr_map)
                print("added obstacle successfully")
                # map_pub.publish(new_map)
                print('New map published')
                curr_map = new_map
            #publish the modified map after adding all the obstacles
            map_pub.publish(curr_map)
        rospy.sleep(0.1)    

if __name__ == '__main__':
    main()
