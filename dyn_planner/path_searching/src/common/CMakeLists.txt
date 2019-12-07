cmake_minimum_required(VERSION 3.2)
project(path_searching)

AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( try_common ${SUB_SOURCES})
ADD_LIBRARY ( try_common  ${try_common})
target_link_libraries(try_common ${catkin_LIBRARIES})
