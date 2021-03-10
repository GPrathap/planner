cmake_minimum_required(VERSION 2.8.3)
project(rebound_opt)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug")


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  path_searching
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rebound_opt
 CATKIN_DEPENDS plan_env path_searching
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( rebound_opt 
    src/uniform_bspline.cpp 
    src/rebound_optimizer.cpp 
    src/gradient_descent_optimizer.cpp
    )
target_link_libraries( rebound_opt
    ${catkin_LIBRARIES} 
    )  
