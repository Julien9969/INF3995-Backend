#!/bin/bash 

source /opt/ros/humble/setup.bash
source /src/app/ros_nodes/install/setup.sh

# --> to LAUNCH MAP MERGE SIM
ros2 launch multirobot_map_merge map_merge.launch.py slam_toolbox:=True known_init_poses:=True use_sim_time:=true &
