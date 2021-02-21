# Copyright 2019 Open Source Robotics Foundation, Inc.
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
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# To save the map use:
# -
# ros2 run nav2_map_server map_saver_cli -f ~/map
# -

bringup_dir = get_package_share_directory('nav2_bringup')
create2_nav_dir = get_package_share_directory('create2_navigation')

nav2_launch_dir = os.path.join(bringup_dir, 'launch')

nav2_params_file = os.path.join(create2_nav_dir, 'params', 'nav2_params.yaml'),


def generate_launch_description():

    # Arguments

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='true'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    # Nodes

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'params_file': ,
            # 'default_bt_xml_filename': ,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/slam_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'params_file': nav2_params_file,
        }.items(),
    )

    # Launch Description
    return LaunchDescription([
        # Arguments
        sim_time_arg,

        # Nodes
        nav2_bringup,
        slam,
    ])
