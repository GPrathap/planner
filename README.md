**To install px4 controller,**

Go to https://github.com/PX4/Firmware.git and follow the instructions to install the firmware and related components 
Add following lines in the `~/.bashrc` (assuming installed location `/home/jobs/Firmware`)

`export path_fmu=/home/jobs/Firmware
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${path_fmu}/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${path_fmu}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${path_fmu}/build/px4_sitl_default
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/geesara/project/casadi/build/casadi
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path_fmu}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path_fmu}/Tools`





**To complete the code:**

1, comment of following lines in  https://github.com/GPrathap/planner/blob/master/drone_path_traj_contol/CMakeLists.txt#L33

`add_executable(drone_path_traj_reg_node src/path_traj_reg.cpp src/path_traj_reg.h)`

`target_link_libraries(drone_path_traj_reg_node yaml-cpp ${catkin_LIBRARIES})`

`add_executable(interactive_rviz_goal_node src/interactive_goal.cpp)`
`target_link_libraries(interactive_rviz_goal_node ${catkin_LIBRARIES})`

Note:This step is needed when it complies first time,   

2. then  catkin_make and uncomment those lines and recomplite the using catkin_make



Run the planner

`roslaunch drone_sim sim.launch 
roslaunch plan_manage  take_off.launch (this is only needed to activate offboard mode, once you start planning stop this node)
roslaunch plan_manage  rviz_px4.launch
roslaunch plan_manage px4_planner.launch
roslaunch plan_manage  reg.launch `

Then, in the GUI you can select the goal point as you like and and have fun:



