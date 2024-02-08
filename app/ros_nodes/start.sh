rm /etc/ros/rosdep/sources.list.d/20-default.list
apt update
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO