---
title: "Object localization using ROS and AprilTag"
categories:
  - Blog
tags:
  - ROS
  - Apriltag
---

This blog introduces how to employ AprilTag markers for localizing a point in 3-D space. This benefits tasks such as navigation, robot control, and augmentated reality. For preparations, you should have a camera ready.

## Driving the camera

We assume that you have adopted a USB camera as video input.

## AprilTag Introduction

AprilTag is a visual fiducial system used in robotics and camera calibration. Targets are created from an ordinary printer, then the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera.

## Installation

[Official website](https://april.eecs.umich.edu/software/apriltag), [GitHub source code](https://github.com/AprilRobotics/apriltag_ros)

You can install the program by simply putting the source code in the workspace, commands as follows:

```ruby
cd workspace/src
git clone https://github.com/AprilRobotics/apriltag_ros
cd ..
catkin_make
```