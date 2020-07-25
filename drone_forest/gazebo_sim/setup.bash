#!/bin/bash
wget https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_nuttx.sh
source ubuntu_sim_nuttx.sh

cd sim
git submodule init
git submodule update
git checkout v1.9.2


git submodule update --init --recursive 
make px4_sitl gazebo

# roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"lo
