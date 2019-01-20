cmake_minimum_required(VERSION 2.8.12)
project(occlusion_potential_field)

###########################################
## ROS上でリンクしたいライブラリのリスト    ##
###########################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  pcl_ros
  sensor_msgs
  nav_msgs
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  pcl_conversions
  autoware_msgs 
  vector_map
)
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

#######################################################
## ROS上で依存しているパッケージのリストを追加すると     ##
## パッケージが変更された時再ビルドされる               ##
#######################################################
catkin_package(CATKIN_DEPENDS 
  pcl_ros
  roscpp 
  std_msgs 
  tf 
  autoware_msgs 
  vector_map 
  sensor_msgs 
  nav_msgs
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  pcl_conversions
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall ${CMAKE_CXX_FLAGS}")

####################################################################################
## CMake関数add_library()は、ビルドする際のライブラリを指定するために使います。        ##
## デフォルトで、catkinは共有ライブラリをビルドします。                               ##
####################################################################################

# Occlusion Potential Field Library
add_library(occlusion_potential_field_lib SHARED
    include/occlusion_potential_field.h
    src/occlusion_potential_field.cpp
)
####################################################################################
## ターゲットを指定する前に、ターゲットのためのリソース、特にヘッダファイルおよび        ##
## ライブラリをどこで見つけることができるのかを指定する必要があります。                 ##
## Include Paths -                                                                ##
##（一般的にC/C++の)コードをビルドするためのヘッダファイルがどこで見つけられるか        ##
####################################################################################

target_include_directories(occlusion_potential_field_lib PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)
target_link_libraries(occlusion_potential_field_lib
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)

add_dependencies(occlusion_potential_field_lib
    ${catkin_EXPORTED_TARGETS}
)

####################################################################################
## 実行可能なターゲットがどのライブラリにリンクするかを指定する                   ##
## これは一般にadd_executable()の呼びだしの後に実行されます。                     ##
## もし ros is not found(rosが見つかりません)と出る場合は、${catkin_LIBRARIES}を  ##
## 追加してください。                                                             ## 
####################################################################################

###########
## Build ##
###########

##################################################
## ノードを追加する処理 ノード実行ファイルを定義##
##################################################
add_executable(occlusion_potential_field
    src/occlusion_potential_field_node.cpp
)
target_include_directories(occlusion_potential_field PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include)

target_link_libraries(occlusion_potential_field
    occlusion_potential_field_lib
    )

install(TARGETS occlusion_potential_field
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)