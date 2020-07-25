cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_converter)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  nodelet
  pluginlib
  )

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp executable
add_executable(point_cloud_converter_node
  src/converter.cpp)
target_link_libraries(point_cloud_converter_node
  ${catkin_LIBRARIES}
  )

## Declare a cpp executable
add_executable(point2_near
  src/point2_near/point2_near_node.cpp
  src/point2_near/pc2_dist_limiter.cpp
  src/point2_near/pc2_dist_limiter.h)
target_link_libraries(point2_near
  ${catkin_LIBRARIES}
  )


## Mark executables and/or libraries for installation
install(TARGETS point_cloud_converter_node
  #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
