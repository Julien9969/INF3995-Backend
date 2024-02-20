#!/bin/bash 

cd /src/app/ros_nodes 
source /opt/ros/humble/setup.bash
chmod +x /src/app/ros_nodes/deploy-backend.sh
/src/app/ros_nodes/deploy-backend.sh 
source /src/app/ros_nodes/install/setup.sh
ros2 launch ros_gz_example_bringup diff_drive.launch.py
