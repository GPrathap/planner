#run sim

DONT_RUN=1 make px4_sitl_default gazebo

# setup Gazebo env and update package path
export path_fmu=/home/op/src/Firmware
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${path_fmu}/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${path_fmu}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${path_fmu}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path_fmu}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path_fmu}/Tools


roslaunch px4 mavros_posix_sitl.launch
cd -

# roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
