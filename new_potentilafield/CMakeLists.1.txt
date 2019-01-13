cmake_minimum_required(VERSION 2.8.3)
project(occlusion)
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
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)


####################################################################################
## CMake関数add_library()は、ビルドする際のライブラリを指定するために使います。   ##
## デフォルトで、catkinは共有ライブラリをビルドします。                           ##
####################################################################################

# Occlusion Library
add_library(occlusion_lib SHARED
    include/occlusion.h
    src/occlusion.cpp
)
####################################################################################
## ターゲットを指定する前に、ターゲットのためのリソース、特にヘッダファイルおよび ##
## ライブラリをどこで見つけることができるのかを指定する必要があります。           ##
## Include Paths -                                                                ##
##（一般的にC/C++の)コードをビルドするためのヘッダファイルがどこで見つけられるか  ##
####################################################################################

target_include_directories(road_occupancy_processor_lib PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)
####################################################################################
## 実行可能なターゲットがどのライブラリにリンクするかを指定する                   ##
## これは一般にadd_executable()の呼びだしの後に実行されます。                     ##
## もし ros is not found(rosが見つかりません)と出る場合は、${catkin_LIBRARIES}を  ##
## 追加してください。                                                             ## 
####################################################################################

target_include_directories(road_occupancy_processor_lib PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)
###############################################################################################
## 実行ファイルがメッセージあるいはサービスを使い、かつメッセージやサービスをビルドする      ##
## パッケージがあるなら、自動的に生成されるメッセージターゲットへの明示的な依存性を作成し、  ##
## 正しい順序でビルドされるようにする必要があります。                                        ##
###############################################################################################

add_dependencies(occulusion ${catkin_EXPORTED_TARGETS})

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


####################################################################
## ターゲットをシステムにインストールして、ターゲットを他で使う   ##
## 
