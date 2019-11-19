cmake_minimum_required(VERSION 3.2)
project(path_searching)



AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)
SET( try_utils ${SUB_SOURCES})
ADD_LIBRARY ( try_utils  ${try_utils} )