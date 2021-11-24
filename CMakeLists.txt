cmake_minimum_required(VERSION 3.0.2)
project(object_detection_apps)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
  # for using darknet message
  darknet_ros_msgs
)


set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp sensor_msgs std_msgs vision_msgs cv_bridge
 DEPENDS
 OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(od_visualizer src/od_visualizer.cpp)
target_link_libraries(od_visualizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(od_depth src/od_depth.cpp)
target_link_libraries(od_depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


