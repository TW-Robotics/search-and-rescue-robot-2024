cmake_minimum_required(VERSION 2.8.3)
project(comehome_pose_tf2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  rospy
  rospy
  tf2_geometry_msgs
  tf2_py
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/LookupTransform.cfg
)

catkin_package(
  CATKIN_DEPENDS rospy tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

