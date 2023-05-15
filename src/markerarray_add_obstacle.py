#!/usr/bin/python

import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

# Global Variables
marker_arr = MarkerArray()

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

    def add_obstacle_square(self, marker, curr_map):

        width = curr_map.info.width
        height = curr_map.info.height
        # o_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]
        x_coord = int(marker.pose.position.x - curr_map.info.origin.position.x/curr_map.info.resolution)
        y_coord = int(marker.pose.position.y - curr_map.info.origin.position.y/curr_map.info.resolution)

        x_scale = int((marker.scale.x / curr_map.info.resolution)/2)
        y_scale = int((marker.scale.y / curr_map.info.resolution)/2)
        # print(str(y_coord - y_scale) + " " + str(y_coord + y_scale))

        #initialize occupancy grid array
        obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]

        for i in range(0, height):
            for j in range(0, width):
                if j>(x_coord - x_scale) and j<(x_coord+x_scale) and i>(y_coord - y_scale) and i<(y_coord+y_scale): 
                    obstacle_map[i][j] = 100
                else:
                    #obstacle_map[i][j] = self.curr_map.data[(height*width-1)-(i+(j*height))]
                    obstacle_map[i][j] = curr_map.data[(height*width-1)-(j+(i*width))]
        
        mod_map = curr_map
        #print(obstacle_map)
        mod_map.data = matrixToArray(height,width,obstacle_map)
        
        return mod_map
    
    def add_obstacle_circle(self, marker, curr_map):

        width = curr_map.info.width
        height = curr_map.info.height
        print(str(height) + " " + str(width))
        # o_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]
        x_coord = int(marker.pose.position.x - curr_map.info.origin.position.x/curr_map.info.resolution)
        y_coord = int(marker.pose.position.y - curr_map.info.origin.position.y/curr_map.info.resolution)

        x_scale = int((marker.scale.x / curr_map.info.resolution)/2)
        # y_scale = int((marker.scale.y / curr_map.info.resolution)/2)
        # print(str(y_coord - y_scale) + " " + str(y_coord + y_scale))

        #initialize occupancy grid array
        obstacle_map = [[-1 for z in range(0, width+1)] for z in range(0, height+1)]

        for i in range(0, height):
            for j in range(0, width):
                # if j>(x_coord - x_scale) and j<(x_coord+x_scale) and i>(y_coord - y_scale) and i<(y_coord+y_scale): 
                if (j-x_coord)*(j-x_coord) + (i-y_coord)*(i-y_coord) <= (x_scale/2)*(x_scale/2):
                    obstacle_map[i][j] = 100
                else:
                    #obstacle_map[i][j] = curr_map.data[(height*width-1)-(i+(j*height))]
                    obstacle_map[i][j] = curr_map.data[(height*width-1)-(j+(i*width))]
        
        mod_map = curr_map
        #print(obstacle_map)
        mod_map.data = matrixToArray(height,width,obstacle_map)
        
        return mod_map
    
    def add_obstacle(self, marker, curr_map):
        if marker.type == 1:
            return self.add_obstacle_square(marker, curr_map)
        elif marker.type == 2 or marker.type == 3:
            return self.add_obstacle_circle(marker, curr_map)
        else: 
            print('error')
            return OccupancyGrid()



def get_marker_arr(data):
    global marker_arr
    marker_arr = data

def main():
    rospy.init_node("voronoi_add_obstacle")
    global marker_arr
    grid = Grid()
    rospy.sleep(1)
    curr_map = grid.old_map
    
    map_pub = rospy.Publisher("/map_edit", OccupancyGrid, queue_size=1)
    rospy.sleep(1)

    rospy.Subscriber("/obstacle_marker_arr", MarkerArray, get_marker_arr)

    while not rospy.is_shutdown():
        new_map = OccupancyGrid()
        if(grid.ready):
            for marker in marker_arr.markers:
                new_map = grid.add_obstacle(marker, curr_map)
                curr_map = new_map
            map_pub.publish(curr_map)
        rospy.sleep(0.1)    

if __name__ == '__main__':
    main()