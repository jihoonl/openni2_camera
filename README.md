openni2_camera
==============

## The fork includes the OpenNI-Linux-x64-2.3 files that work with the original Orbbec Astra

  It would be better if I provided a link to where I downloaded the OpenNI-Linux-x64-2.3.zip
  for that particular SDK version that I have included, but in the various testing and frantic debugging trying to bring the Astra to life
  under ROS 2 that information has been lost :-(

  This works, where the Ubuntu 20.04 OpenNI version was not recognizing the camera.
  
==============

ROS2 wrapper for openni 2.0

Note: openni2_camera supports xtion devices, but not Kinects.

## Running ROS2 Driver

An example launch exists that loads just the camera component:

```
ros2 launch openni2_camera camera_only.launch.py
```

If you want to get a PointCloud2, use:

```
ros2 launch openni2_camera camera_with_cloud.launch.py
```

## Migration from ROS1

 * The GetService message has moved to a new openni2_camera_msgs package.
 * The rgb/image topic has been renamed to rgb/image_raw for consistency.
 * The nodelet has been refactored into an rclcpp component called
   "openni2_wrapper::OpenNI2Driver". See the launch folder for an example
   of how to start this.
 * Since most components in image_proc/depth_image_proc lack lazy pub/sub,
   the advanced processing graphs in rgbd_launch and openni2_launch are not
   currently feasible. It is recommended to create a launch file with the
   specific pipeline you want. See the launch folder for an example.

## Known Issues

 * There are currently no subscriber connect/disconnect callbacks in ROS2.
   This package implements a lazy publisher by running a 1Hz update loop
   and seeing if there are new subscribers.
 * Using "use_device_time" is currently broken.
