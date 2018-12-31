cmake_minimum_required(VERSION 2.8.3)
project(new_potentialfield)

###########################################
## ROS上でリンクしたいライブラリのリスト ##
###########################################
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
###########################################
## ROS上でリンクしたいライブラリのリスト ##
###########################################


#######################################################
## ROS上で依存しているパッケージのリストを追加すると ##
## パッケージが変更された時、再ビルドされる          ##
#######################################################
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
##################################################
## ノードを追加する処理 ノード実行ファイルを定義##
##################################################
add_executable(occlusion nodes/occlusion.cpp)
target_link_libraries(occlusion ${catkin_LIBRARIES})
add_dependencies(occulusion ${catkin_EXPORTED_TARGETS})

