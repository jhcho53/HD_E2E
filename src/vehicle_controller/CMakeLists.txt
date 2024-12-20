cmake_minimum_required(VERSION 3.0.2)
project(vehicle_controller)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  geometry_msgs
  rospy
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS morai_msgs geometry_msgs rospy std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Python script installation
catkin_install_python(PROGRAMS
  scripts/vehicle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

