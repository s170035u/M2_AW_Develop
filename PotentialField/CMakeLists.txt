cmake_minimum_required(VERSION 2.8.3)
project(coi_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  pcl_ros
  pcl_conversions
  waypoint_follower
  autoware_msgs 
  vector_map
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf waypoint_follower autoware_msgs vector_map
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offset_operation nodes/offset_operation_node.cpp)
target_link_libraries(offset_operation ${catkin_LIBRARIES})
add_dependencies(offset_operation ${catkin_EXPORTED_TARGETS})

