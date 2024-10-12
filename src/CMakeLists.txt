cmake_minimum_required(VERSION 3.0.2)
project(simple_planner_rp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
  cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(node 
  /home/lattinone/Desktop/simple_planner_rp/src/node.cpp 
  /home/lattinone/Desktop/simple_planner_rp/src/RViz.cpp 
  /home/lattinone/Desktop/simple_planner_rp/src/path_finder.cpp 
  /home/lattinone/Desktop/simple_planner_rp/src/map_manag.cpp 
)
target_link_libraries(node ${catkin_LIBRARIES})
