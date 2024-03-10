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
# Delay 10 seconds to get away from the robot
# sleep 10

# Start tmux. first window is roscore, second is urg_node, third is depthai and fourth is vilo, fith is rosbag record
# tmux new-session -d -s vilo -n 'vilo'
# tmux send-keys -t vilo:0 'rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map laser 10' C-m
# tmux split-window -h
# # Start lidar driver
# tmux send-keys -t vilo:1 'rosrun urg_node urg_node' C-m
# tmux split-window -v
# # Start depthai oak-d
# tmux send-keys -t vilo:2 'roslaunch depthai_examples stereo_node.launch' C-m
# tmux split-window -v
# # Undistort
# tmux send-keys -t vilo:3 'ROS_NAMESPACE=/stereo_publisher/ rosrun stereo_image_proc stereo_image_proc' C-m
# tmux split-window -v
# # Start vilo
# tmux send-keys -t vilo:4 'roslaunch vilo vilo.launch' C-m
# tmux split-window -v
# # Start bag recording
# tmux send-keys -t vilo:5 'rosbag record -a -O /home/robo/recorded_data.bag' C-m

# Send route to pioneers with p2os driver
