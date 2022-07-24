#!/usr/bin/env python3

import os

import launch
import launch_ros.actions
import launch_ros.descriptions
def generate_launch_description():
    tf_node_mini = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        #arguments=["0 0 0 1.57 0 0 base mini/openni_rgb_optical_frame"])
        arguments=["0 0 0 0 0 0 base mini"])
    """
    tf_node_orbbec = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0 0 0 -1.57 0 0 base orbbec/openni_rgb_optical_frame"])
    """
    ld = launch.LaunchDescription([tf_node_mini])
    return ld
