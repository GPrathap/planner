cmake_minimum_required(VERSION 2.8.3)
project(interactive_goal)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  drone_msgs
  geometry_msgs
  interactive_markers
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## catkin specific configuration 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interactive_goal
#  CATKIN_DEPENDS drone_reg geometry_msgs interactive_marker nav_msgs roscpp std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(interactive_posestamped_node src/interactive_posestamped.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(interactive_posestamped_node
  ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable(interactive_goal_node src/interactive_posestamped.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(interactive_goal_node
  ${catkin_LIBRARIES}
)




