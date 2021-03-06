cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

add_definitions(-std=c++11)


set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC -fopenmp -DBOOST_LOG_DYN_LINK")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
)

find_package(Boost COMPONENTS program_options log log_setup regex atomic date_time system chrono filesystem thread graph REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
include_directories(${BOOST_INCLUDE_DIRS})
include_directories(/usr/local/lib/)
link_directories(/usr/local/lib/)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

message( STATUS "Include sub-directories into build process:" )
foreach( DIR rrt_star common utils)
    message( STATUS "  - include/" ${DIR} )
    include_directories( "src/${DIR}" )
    link_directories("src/${DIR}")
    add_subdirectory( "src/${DIR}" )
endforeach()


set(TRACKING_ESTIMATION_LIB ${TRACKING_ESTIMATION_LIB} ${Boost_LIBRARIES})
set(TRACKING_ESTIMATION_LIB  ${TRACKING_ESTIMATION_LIB} try_rrt_star try_common try_utils cnpy z)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O2 -Wall")
add_library( path_searching  src/kinodynamic_rrt_star.cpp )

target_link_libraries(path_searching -Wl,--start-group  
                                ${TRACKING_ESTIMATION_LIB} -Wl,--end-group)
target_link_libraries( path_searching ${catkin_LIBRARIES})  
