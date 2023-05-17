---
title: "Object localization using ROS and AprilTag"
categories:
  - Blog
tags:
  - ROS
  - Apriltag
---

This blog introduces how to employ AprilTag markers for localizing a target in 3-D space. This benefits tasks such as navigation, robot control, and augmentated reality. For preparations, you should have a camera ready.

# Lauching the camera

In our design, we connect a USB camera with the Raspberry Pi as the system's visual input. We launch the camera video feed as ROS topics for further processing. Installation commands of the program as follows:

```ruby
cd workspace/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
cd ~/workspace
catkin_make
```

Note that errors may occur in the **catkin_make** process, which is probabily due to lack of relevant packages in the environment. Solutions are across the web.

To launch the camera as ROS ropics, input the following command. The published topics surrounded by a red box can be examined.

```ruby
source devel/setup.bash
roslaunch usb_cam usb_cam-test.launch
```

![camera-topic](/assets/images/rosapriltag-camera-topic.png)

# Running AprilTag Detection Algorithm

AprilTag is a visual fiducial system used in robotics and camera calibration. Targets are created from an ordinary printer, then the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera. You can find some of the AprilTag (of tag family 36h11)at this [link](https://www.dotproduct3d.com/uploads/8/5/1/1/85115558/apriltags1-20.pdf).

## Installation

[Official website](https://april.eecs.umich.edu/software/apriltag), [GitHub source code](https://github.com/AprilRobotics/apriltag_ros)

Installation of the program is simply putting the source code in the workspace, commands as follows:

```ruby
cd workspace/src
git clone https://github.com/AprilRobotics/apriltag_ros
cd ~/workspace
catkin_make
```