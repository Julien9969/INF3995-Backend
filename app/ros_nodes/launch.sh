rm /etc/ros/rosdep/sources.list.d/20-default.list
apt update
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.sh
ros2 launch py_identify_server identify.py