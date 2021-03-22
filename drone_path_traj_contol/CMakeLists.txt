cmake_minimum_required(VERSION 2.8.3)
project(drone_path_traj_contol)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  interactive_markers
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  mavros_msgs
  quadrotor_msgs
)

find_package(yaml-cpp)

generate_dynamic_reconfigure_options(
         cfg/DroneReg.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs interactive_markers nav_msgs roscpp rospy std_msgs quadrotor_msgs tf visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drone_path_traj_reg_node src/path_traj_reg.cpp src/path_traj_reg.h)

target_link_libraries(drone_path_traj_reg_node yaml-cpp ${catkin_LIBRARIES})

add_executable(interactive_rviz_goal_node src/interactive_goal.cpp)
target_link_libraries(interactive_rviz_goal_node ${catkin_LIBRARIES})
