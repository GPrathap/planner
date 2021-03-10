cmake_minimum_required(VERSION 2.8.3)
project(plan_env)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  octomap_msgs
  octomap_ros
  traj_utils
)


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

#set(octomap_ROOT_DIR /usr/local/)
#find_package(octomap REQUIRED)
include_directories(/usr/local/include/)
#include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(/usr/local/lib/)
#target_link_libraries(${OCTOMAP_LIBRARIES})

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs octomap_ros traj_utils
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env
    src/edtoctomap.cpp
    src/edt_environment.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} octomap dynamicedt3d
    )
