### Real-Time Path Planning using Voronoi Graphs

# Dependencies
The framework consisted of an Intel Realsense mounted on a Firebird VI robot.
ROS_VERSION: kinetic
tuw_multi_robot package: https://github.com/tuw-robotics/tuw_multi_robot

# How to Run
From the catkin_ws folder, execute the following commands:
```linux
roslaunch rail_segmentation rail_segmentation.launch
rosservice call /rail_segmentaion/segment "{}"
roslaunch tuw_voronoi_graph corridor_voronoi.launch
rostopic pub -r 
```
