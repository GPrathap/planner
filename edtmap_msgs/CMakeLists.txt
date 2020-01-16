cmake_minimum_required(VERSION 2.8.3)
project(edtmap_msg)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_message_files(DIRECTORY msg FILES EDTMap.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)


include_directories(include  ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(edtmap_msg src/edtmap_msg.cpp)
target_link_libraries(edtmap_msg ${catkin_LIBRARIES})
add_dependencies(edtmap_msg edtmap_msg_generate_messages_cpp)

install(TARGETS edtmap_msg
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" )
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )