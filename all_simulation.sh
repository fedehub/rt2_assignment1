#!/bin/bash
gnome-terminal --tab --title="roscore" -- bash -c "source ros.sh; roscore"
gnome-terminal --tab --title="ros_assignment" -- bash -c "sleep 2; source ros.sh; roslaunch rt2_assignment1 sim2.launch"
gnome-terminal --tab --title="bridge" -- bash -c "source ros12.sh; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"
gnome-terminal --tab --title="ros2_assignment" -- bash -c "source ros2.sh; ros2 launch rt2_assignment1 all_launch.py"
