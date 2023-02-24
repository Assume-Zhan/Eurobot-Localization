cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(
  catkin
  REQUIRED
  COMPONENTS 
  roscpp
  roslaunch
  tf2
  tf2_ros
  nodelet
  rviz
  std_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  obstacle_detector
)

find_package(Armadillo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_nodelets beacon_scan_tracker area_obstacles_extractor
  CATKIN_DEPENDS roscpp rviz std_msgs geometry_msgs sensor_msgs tf2 tf2_ros laser_geometry nodelet obstacle_detector
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

#
# Build libs
#
add_library(lidar_localization src/lidar_localization.cpp)
target_link_libraries(lidar_localization ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(lidar_localization ${catkin_EXPORTED_TARGETS})

add_library(area_obstacles_extractor src/area_obstacles_extractor.cpp)
target_link_libraries(area_obstacles_extractor ${catkin_LIBRARIES})
add_dependencies(area_obstacles_extractor ${catkin_EXPORTED_TARGETS})

add_library(beacon_scan_tracker src/beacon_scan_tracker.cpp)
target_link_libraries(beacon_scan_tracker ${catkin_LIBRARIES})
add_dependencies(beacon_scan_tracker ${catkin_EXPORTED_TARGETS})

#
# Build nodes
#
add_executable(lidar_localization_node src/nodes/lidar_localization_node.cpp)
target_link_libraries(lidar_localization_node lidar_localization)

add_executable(area_obstacles_extractor_node src/nodes/area_obstacles_extractor_node.cpp)
target_link_libraries(area_obstacles_extractor_node area_obstacles_extractor)


add_executable(beacon_scan_tracker_node src/nodes/beacon_scan_tracker_node.cpp)
target_link_libraries(beacon_scan_tracker_node beacon_scan_tracker)

#
# Build nodelets
#
add_library(${PROJECT_NAME}_nodelets src/nodelets/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelets ${PROJECT_NAME})