cd /src/app/ros_nodes
rm /etc/ros/rosdep/sources.list.d/20-default.list
source /opt/ros/humble/setup.bash
rosdep init
rosdep update
rosdep install --from-paths /src/app/ros_nodes/src --ignore-src -r -i -y --rosdistro humble
sleep 5
source /src/app/ros_nodes/install/setup.sh
colcon build
source /src/app/ros_nodes/install/setup.sh
