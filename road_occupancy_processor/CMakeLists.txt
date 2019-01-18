cmake_minimum_required(VERSION 2.8.12)
project(road_occupancy_processor)

find_package(catkin REQUIRED COMPONENTS
    autoware_build_flags
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
    roscpp
    tf
    sensor_msgs
    nav_msgs
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    vector_map
)

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

catkin_package(CATKIN_DEPENDS
    roscpp
    tf
    sensor_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    vector_map
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

#Road Occupancy Processor Library
add_library(road_occupancy_processor_lib SHARED
    include/road_occupancy_processor.h
    src/road_occupancy_processor.cpp
)
################################################################################################
## 非常に重要なこととして、catkinの中のビルドターゲットの名前は、それらがビルド/インストール  ##
## されるフォルダを問わず一意でなければならないということです。これはCMakeの要件です。しかし  ##
## ながら、ターゲットの一意な名前付けが必要なのはCMakeの内部だけです。set_target_properties() ##
## 関数を使って、ターゲットを他の別の名前にして保持することができます：                       ##
## これはビルドやインストール時の出力ターゲットの名前をrviz_image_view から image_viewへ変更  ##
################################################################################################
if (OPENMP_FOUND)
    set_target_properties(road_occupancy_processor_lib PROPERTIES
        COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
        LINK_FLAGS ${OpenMP_CXX_FLAGS}
    )
endif()

target_link_libraries(road_occupancy_processor_lib
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)

target_include_directories(road_occupancy_processor_lib PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)

add_dependencies(road_occupancy_processor_lib
    ${catkin_EXPORTED_TARGETS}
)

#Occupancy Processor Main
add_executable(road_occupancy_processor
    src/road_occupancy_processor_node.cpp
)
target_include_directories(road_occupancy_processor PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include)

target_link_libraries(road_occupancy_processor
    road_occupancy_processor_lib)

install(TARGETS road_occupancy_processor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
