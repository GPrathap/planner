cmake_minimum_required(VERSION 2.8.3)
project(mpc_opt)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  traj_utils
  bspline_opt
  rebound_opt
  xmlrpcpp
)


include_directories(/usr/local/include/)
link_directories(/usr/local/lib/)

set(EXTERNAL_LIBS cnpy z casadi)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mpc_opt
 CATKIN_DEPENDS  plan_env traj_utils bspline_opt rebound_opt xmlrpcpp
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( mpc_opt src/nonlinear_mpc_opt.cpp src/kalman.cpp src/trajectory_tracker.cpp 
            src/trajectory_regulator.cpp src/bspline_utils.cpp src/trajectory_tracker_validate.cpp )
target_link_libraries(mpc_opt -Wl,--start-group  
                                ${EXTERNAL_LIBS} -Wl,--end-group)
