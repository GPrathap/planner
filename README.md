**To install px4 controller,**

Go to https://github.com/PX4/Firmware.git and follow the instructions to install the firmware and related components 
Add following lines in the `~/.bashrc` (assuming installed location `/home/jobs/Firmware`)

`export path_fmu=/home/jobs/Firmware
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${path_fmu}/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${path_fmu}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${path_fmu}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path_fmu}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path_fmu}/Tools`





**To compile the code:**


Run the planner

`roslaunch drone_sim sim.launch `

`roslaunch plan_manage  take_off.launch` (this is only needed to activate offboard mode, once you start planning stop this node)

`roslaunch plan_manage  rviz_px4.launch`

`roslaunch plan_manage px4_planner.launch`

`roslaunch plan_manage  reg.launch `

Then, in the GUI you can select the goal point as you like and and have fun:



