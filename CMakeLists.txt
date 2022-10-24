cmake_minimum_required(VERSION 3.0.2)
project(ros_to_pix4d)

set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  rospy  
  roscpp
  roslib
  # libraries
  cv_bridge
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(OpenCV)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS OpenCV

  CATKIN_DEPENDS
  roslib
  cv_bridge 
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs 
)

# include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# src
add_executable(${PROJECT_NAME}_ROSToPix4D src/ROSToPix4D.cpp)
target_link_libraries(${PROJECT_NAME}_ROSToPix4D ${catkin_LIBRARIES})
