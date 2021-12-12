
# TurtleBot3 Lane Following

![](https://img.shields.io/badge/Python-2.7-yellow)
![](https://img.shields.io/badge/ROS-melodic-blue)
![](https://img.shields.io/badge/Ubuntu-18.04-orange)
![](https://img.shields.io/badge/OpenCV-3.2-red)

Lane tracking and controlling using perception. The lane is from TurtleBot3 autorace mission consists of yellow lane on the left and white lane on the right.

## Prerequisites

- [TurtleBot3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)

- [Raspicam node](https://github.com/UbiquityRobotics/raspicam_node)

- [ROS Melodic & TurtleBot3 ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

## Installation

Open the terminal and run command below to download and install on `Remote PC` and `SBC`. If [TurtleBot3 AutoRace 2020 metapackage](https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020) branch kinetic is installed, skip it:

    $ cd ~/catkin_ws/src/
    $ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
    $ git clone https://github.com/VIBOT-ROS/turtlebot3_lane_following.git
    $ cd ~/catkin_ws/turtlebot3_lane_following/scripts/ && chmod +x * && cd ~/catkin_ws && catkin_make

Install additional dependent packages on `Remote PC`:

    $ sudo apt-get install ros-melodic-image-transport ros-melodic-cv-bridge ros-melodic-vision-opencv python-opencv libopencv-dev ros-melodic-image-proc

## Calibration

To run the lane following, first thing is to calibrate the camera on TurtleBot3. The raspicam_node package is used. Execute below on the terminal

#### Camera Calibration

Install [camera calibration](http://wiki.ros.org/camera_calibration) package if haven't installed yet.

  1. First launch roscore on `Remote PC`:

    $ roscore

  2. Next launch raspberry pi camera on `SBC`:

    $ roslaunch turtlebot3_lane_following turtlebot3_camera_pi.launch

  3. Then launch camera calibrator on `Remote PC`:

    $ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/my_camera/image camera:=/my_camera

  4. After finish calibration. Copy and paste the data inside the **ost.yaml** to **camerav2_320x240_30fps.yaml**

  5. Closed camera calibration node & raspberry pi camera node by `ctrl-c`.

  6. Relaunch raspberry pi camera to apply rectification or undistort image.

#### Lane Detection Calibration

The lane detection needs to be adjust or tuning in order to detect both lanes.

## One Launch to run all the nodes

Launch one to rule them all


