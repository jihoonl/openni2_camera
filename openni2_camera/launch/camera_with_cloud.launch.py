#!/usr/bin/env python3

# Copyright (c) 2020, Michael Ferguson
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

import launch
import launch_ros.actions
import launch_ros.descriptions


def rgbd_camera_container(device_id, namespace):

    container = launch_ros.actions.ComposableNodeContainer(
        name='container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Driver
            launch_ros.descriptions.ComposableNode(
                package='openni2_camera',
                plugin='openni2_wrapper::OpenNI2Driver',
                name='driver',
                namespace=namespace,
                parameters=[{
                    'depth_registration': True,
                    'device_id': device_id,
                    'frame_prefix': namespace
                }, {
                    'use_device_time': False
                }],
                # remappings=[('depth/image_raw', 'depth_registered/image_raw')],
            ),
            # Create XYZRGB point cloud
            launch_ros.descriptions.ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='points_xyzrgb',
                namespace=namespace,
                parameters=[{
                    'queue_size': 20
                }],
                remappings=[
                    ('rgb/image_rect_color', 'rgb/image_raw'),
                    ('rgb/camera_info', 'rgb/camera_info'),
                    ('depth_registered/image_rect', 'depth/image'),
                    ('points', 'depth_registered/points'),
                ],
            ),
        ],
        output='screen',
    )
    return container


def generate_launch_description():
    container_mini = rgbd_camera_container('17', 'mini')
    container_orbbec = rgbd_camera_container('19', 'orbbec')
    tf_node_mini = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=list(map(str, [0, 0, -1, 0, 0, -1.57, 'base', 'mini/openni_rgb_optical_frame'])))
    tf_node_orbbec = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=list(map(str, [0, 0, 1., 0, 0, -1.57, 'base', 'orbbec/openni_rgb_optical_frame'])))

    ld = launch.LaunchDescription([
        launch.actions.TimerAction(period=1.0,
                                   actions=[tf_node_mini, tf_node_orbbec]),
        launch.actions.TimerAction(period=2.0, actions=[container_mini]),
        launch.actions.TimerAction(period=3.0, actions=[container_orbbec]),
    ])
    return ld
