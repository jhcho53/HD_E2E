cmake_minimum_required(VERSION 3.0.2)
project(global_path_demo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)

## Declare catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(global_path_from_file src/global_path_from_file.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(global_path_from_file
  ${catkin_LIBRARIES}
)
