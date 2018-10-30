cmake_minimum_required(VERSION 2.8.3)
project(kinematics_animation)

## find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbag
  roscpp
  rospy
  robot_state_publisher
  joint_state_publisher
  urdf
)

## catkin specific configuration
catkin_package()

## build
include_directories(
  ${catkin_INCLUDE_DIRS}
)
