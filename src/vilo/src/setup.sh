#!/bin/bash
#todo check for required packages
#TODO move vilo launch into psos_launch then make symlink back for editing
# create symbolic link to launch file from /opt/ros/noetic/share/p2os_launch/launch/p2os_oakd_hokuyo.launch to current directory
# ln -s /opt/ros/noetic/share/p2os_launch/launch/p2os_oakd_hokuyo.launch ~/catkin_ws/src/vilo/launch/p2os_oakd_hokuyo.launch
current_dir=$(pwd)
# if --remake is passed, delete the catkin_ws and remake it
if [ "$1" == "--remake" ]; then
    echo "Deleting catkin_ws and remaking it"
    rm -rf ~/catkin_ws/build ~/catkin_ws/devel ~/catkin_ws/install
    cd ~/catkin_ws
    catkin_make
fi
cd ~/catkin_ws
catkin_make

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# password is "CSCI-632"
echo "CSCI-632" | sudo -S chmod a+rw /dev/ttyACM0 # Hokuyo lidar
echo "CSCI-632" | sudo -S chmod a+rw /dev/ttyUSB0 # Pioneer driver

# kill any tmux sessions that are running
tmux kill-server

echo "Setup complete"
cd $current_dir
