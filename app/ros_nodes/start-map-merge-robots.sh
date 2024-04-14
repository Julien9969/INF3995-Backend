#!/bin/bash 

source /opt/ros/humble/setup.bash
source /src/app/ros_nodes/install/setup.sh

# --> to LAUNCH MAP MERGE VRAIS ROBOT A TESTER
ros2 launch multirobot_map_merge map_merge.launch.py slam_toolbox:=False known_init_poses:=True use_sim_time:=false
