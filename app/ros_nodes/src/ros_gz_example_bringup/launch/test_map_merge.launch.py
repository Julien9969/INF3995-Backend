# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    map_merge_dir = os.path.join(get_package_share_directory('multirobot_map_merge'), 'launch')

    return LaunchDescription([
        #############################
        # Map Merge Nodes and utils!
        #############################

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([map_merge_dir, '/map_merge.launch.py']),
            launch_arguments={
                "slam_toolbox": "True",
                "known_init_poses": "False",
                "use_sim_time": "false",
            }.items(),
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=[
                '0', '0', '0', '0', '0', '0', '1', 'world', 'robot1/map'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=[
                '0', '0', '0', '0', '0', '0', '1', 'world', 'robot2/map'
            ],
        ),


    ])
