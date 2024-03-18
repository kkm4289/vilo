#!/bin/bash
#TODO check for required packages
#TODO arguments should not be strictly ordered
#TODO bad practice to modify .bashrc directly

if [ "$1" == "--terminal" ]; then
    echo "New terminal. Just sourcing :)"
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
fi

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

if [ "$2" != "--dry" ]; then
# password is "CSCI-632"
echo "CSCI-632" | sudo -S chmod a+rw /dev/ttyACM0 # Hokuyo lidar
echo "CSCI-632" | sudo -S chmod a+rw /dev/ttyUSB0 # Pioneer driver
fi


# kill any tmux sessions that are running
tmux kill-server > /dev/null 2>&1

echo "Setup complete"
cd $current_dir


# check if in docker
# if in docker and loop variable != true
if [ -f /.dockerenv ]; then
    echo "In docker"
    # apt update && apt upgrade -y >> /dev/null
    # add to .bashrc to run this script on startup of each terminal
    echo "source ~/catkin_ws/devel/setup.bash --terminal" >> ~/.bashrc
else
    echo "Not in docker"
fi