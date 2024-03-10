#!/bin/bash

#TODO add battery check and warning
#get next number bag name 
# bag_number=$(ls -1 /home/rnd_lab/catkin_ws/src/vilo/data* | wc -l)
# bag_number=$((bag_number+1))
# bag_name="/home/rnd_lab/catkin_ws/src/vilo/data/bag$bag_number"
# echo "Recording to $bag_name"

tmux kill-server > /dev/null 2>&1
tmux new-session -d -s vilo -n 'vilo'

# Start roscore
# tmux send-keys -t vilo:1.1 'roscore' C-m
tmux split-window -v
tmux send-keys -t vilo:1.2 'roslaunch vilo p2os_oakd_hokuyo.launch' C-m
echo "Started p2os_oakd_hokuyo.launch"


tmux split-window -h
# record all topics
# bag_name="/home/rnd_lab/catkin_ws/src/vilo/data/b1"

tmux send-keys -t 1.3 'rosbag record -O "/home/rnd_lab/catkin_ws/src/vilo/data/b2.bag" /scan /pose /stereo_publisher/left/image /stereo_publisher/right/image /stereo_publisher/stereo/depth /stereo_publisher/left/camera_info /stereo_publisher/right/camera_info' C-m

tmux split-window -v
# tmux send-keys -t 1.4 'python3 /home/rnd_lab/catkin_ws/src/beginner_tutorials/scripts/goto0.py /home/rnd_lab/catkin_ws/src/beginner_tutorials/scripts/more_points' C-m

# time for robot to move
sleep 10


# stop bag
tmux send-keys -t vilo:1.3 C-c
sleep 5
echo "Recording complete"

# record pose
# tmux send-keys -t vilo:1.3 'rostopic echo -p /pose > "b1_pose.txt" C-m
# echo "Recorded pose"



tmux kill-session -t vilo
