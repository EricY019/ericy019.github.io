---
title: "Object localization using ROS and AprilTag"
categories:
  - Blog
tags:
  - ROS
  - Apriltag
---

This blog shares one of the major achievements of our final year project. Our objective is to develop a system capable of **recognizing elevator buttons using a monocular camera** and **instructing the robotic arm to press the button**. AprilTag is particularly useful in this task as it well supports ROS.

## AprilTag Introduction

AprilTag is a visual fiducial system used in robotics and camera calibration. Targets are created from an ordinary printer, then the AprilTag detection software computes the precise 3D position, orientation, and identity of the tags relative to the camera.

## Installation

[Official website](https://april.eecs.umich.edu/software/apriltag)

[Source code GitHub](https://github.com/AprilRobotics/apriltag_ros)

You can install the program by simply putting the source code in the workspace, commands as follows:

```ruby
cd workspace/src
git clone https://github.com/AprilRobotics/apriltag_ros
cd ..
catkin_make
```