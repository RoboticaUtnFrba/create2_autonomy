#!/usr/bin/env python

# Copyright 2021 Emiliano Javier Borghi Orue.
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

"""Launch Webots iRobot Create 2 driver."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # launch plugin through rclcpp_components container
    depth_image_proc = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Driver itself
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/camera_info', '/camera/camera_info'),
                    ('rgb/image_rect_color', '/camera/image_raw'),
                    ('depth_registered/image_rect', '/camera/image_depth'),
                    ('points', '/camera/depth_registered/points')
                ]
            ),
        ],
        output='both',
    )

    return LaunchDescription([
        # Nodes
        depth_image_proc,
    ])
