#!/bin/bash 

cd /src/app/ros_nodes 
source /opt/ros/humble/setup.bash
chmod +x /src/app/ros_nodes/deploy-backend.sh
/src/app/ros_nodes/deploy-backend.sh 
source /src/app/ros_nodes/install/setup.sh

# TODO:
# --> to LAUNCH MAP MERGE SIM
# ros2 launch multirobot_map_merge map_merge.launch.py slam_toolbox:=True known_init_poses:=True use_sim_time:=true &
# --> to LAUNCH MAP MERGE VRAIS ROBOT A TESTER
# ros2 launch multirobot_map_merge map_merge.launch.py slam_toolbox:=False known_init_poses:=True use_sim_time:=false &

ros2 launch ros_gz_example_bringup diff_drive.launch.py
