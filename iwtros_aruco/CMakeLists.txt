cmake_minimum_required(VERSION 2.8.3)
project(iwtros_aruco)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED )


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    aruco_detection
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    tf2
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sample_cv_bridge_node 
  src/sample_cv_bridge.cpp
)
target_link_libraries(sample_cv_bridge_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

 add_executable(aruco_calibration_node 
  src/aruco_calibrate.cpp
)
target_link_libraries(aruco_calibration_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

 # Create ditect aruco library
add_library(aruco_detector
  src/aruco_detection.cpp
)
target_link_libraries(aruco_detector 
  ${catkin_LIBRARIES}  
  ${OpenCV_LIBRARIES}
)

add_executable(marker_detection_node 
  src/marker_detection_node.cpp
)
target_link_libraries(marker_detection_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   aruco_detector
 )