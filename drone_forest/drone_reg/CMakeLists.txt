cmake_minimum_required(VERSION 2.8.3)
project(drone_reg)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslib
  drone_msgs
  geometry_msgs
  dynamic_reconfigure
  mavros_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)
find_package(yaml-cpp)

## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()



## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/DroneReg.cfg
)


## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drone_reg
#  CATKIN_DEPENDS geometry_msgs mavros_msgs rospy sensor_msgs std_msgs tf
   CATKIN_DEPENDS message_runtime dynamic_reconfigure roslib
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(drone_reg_cpp
    src/drone_reg_node.h
    src/drone_reg_node.cpp)
target_link_libraries(drone_reg_cpp yaml-cpp
  ${catkin_LIBRARIES}
  )


catkin_install_python(PROGRAMS bin/drone_reg_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

