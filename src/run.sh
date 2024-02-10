#!/bin/bash

source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash

# Delay 10 seconds to get away from the robot
sleep 10

# Start tmux. first window is roscore, second is urg_node, third is depthai and fourth is morvilo, fith is rosbag record
tmux new-session -d -s MoRViLO -n 'MoRViLO'
tmux send-keys -t MoRViLO:0 'roscore' C-m
tmux split-window -h
# Start lidar driver
tmux send-keys -t MoRViLO:1 'rosrun urg_node urg_node' C-m
tmux split-window -v
# Start depthai oak-d
tmux send-keys -t MoRViLO:2 'roslaunch depthai_examples stereo_node.launch' C-m
tmux split-window -v
# Undistort
tmux send-keys -t MoRViLO:3 'ROS_NAMESPACE=/stereo_publisher/ rosrun stereo_image_proc stereo_image_proc' C-m
tmux split-window -v
# Start MoRViLO
tmux send-keys -t MoRViLO:4 'roslaunch morvilo morvilo.launch' C-m
tmux split-window -v
# Start bag recording
tmux send-keys -t MoRViLO:5 'rosbag record -a -O /home/robo/recorded_data.bag' C-m

# Send route to pioneers with p2os driver

