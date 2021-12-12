
# TurtleBot3 Lane Following

![](https://img.shields.io/badge/Python-2.7-yellow)
![](https://img.shields.io/badge/ROS-melodic-brightgreen)
![](https://img.shields.io/badge/Ubuntu-18.04-orange)
![](https://img.shields.io/badge/OpenCV-3.2-blue)

Lane tracking and controlling using perception. The lane is from TurtleBot3 autorace mission consists of yellow lane on the left and white lane on the right.

## Prerequisites

- [TurtleBot3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)

- [Raspicam node](https://github.com/UbiquityRobotics/raspicam_node)

- [ROS Melodic & TurtleBot3 ROS packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)

## Installation

Open the terminal and run command below to download and install on `Remote PC` and `SBC`. If [TurtleBot3 AutoRace 2020 metapackage](https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020) branch kinetic is installed, skip it:

    cd ~/catkin_ws/src/
    git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
    git clone https://github.com/VIBOT-ROS/turtlebot3_lane_following.git
    cd ~/catkin_ws/turtlebot3_lane_following/scripts/ && chmod +x * && cd ~/catkin_ws && catkin_make

Install additional dependent packages on `Remote PC`:

    sudo apt-get install ros-melodic-image-transport ros-melodic-cv-bridge ros-melodic-vision-opencv python-opencv libopencv-dev ros-melodic-image-proc

## Calibration

To run the lane following, first thing is to calibrate the camera on TurtleBot3. The raspicam_node package is used. Execute below on the terminal

#### Camera Calibration

Install [camera calibration](http://wiki.ros.org/camera_calibration) package if haven't installed yet.

  1. First launch roscore on `Remote PC`:

    roscore

  2. Next launch raspberry pi camera on `SBC`:

    roslaunch turtlebot3_lane_following turtlebot3_camera_pi.launch

  3. Then launch camera calibrator on `Remote PC`:

    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image camera:=/camera

  4. After finish calibration. Go to **/tmp/calibrationdata.tar.gz** and extract. Copy and paste the `data` inside the **ost.yaml** to **/home/pc_username/catkin_ws/src/turtlebot3_lane_following/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml**

  5. Closed the camera calibration node & raspberry pi camera node by `ctrl-c`.

  6. Relaunch raspberry pi camera to apply rectification or undistort image.

#### Lane Detection Calibration

launch the detection on `Remote PC`:

    roslaunch turtlebot3_lane_following turtlebot3_lane_detection.launch

The lane detection image is publish at topic `/camera/mask_lane_detected/compressed`. To view it run below.

    rqt_image_view
    
Then select the topic **or** run below:
    
    rosrun image_view image_view image:=/camera/mask_lane_detected/compressed
    
If it does not detect both lane, `ctrl-c` the `lane_detection.launch` and go to directory `/home/your_username/catkin_ws/src/turtlebot3_lane_following/scripts/lane_detection.py` and open the scripts and edit `thresh = cv2.inRange(gray, 190, 255)` value between 0 to 255. In this case is 190 and 255. Means pixel value between 190 to 255 will be white(detected) and the rest is black(undetected). Then run again `lane_detection.launch`. repeat the process until it detect both lanes.

## Controller


Launch below to run the controller on `Remote PC`:

    roslaunch turtlebot3_lane_following turtlebot3_lane_controller.launch

#### How the controller works


The PD controller were implemented since is most suitable. Below is derivative from positional PD controller to velocity PD controller. Where `Kp` is proportional gain, `kd` is derivative gain, `e(t)` is error, and `u(t)` is the controller output. The error is the pixel distance between `look ahead` detect lane of a pixel point to a pixel image center. 


  PD controller:

  <img src="https://latex.codecogs.com/svg.image?u(t)=K_{p}*e(t)&plus;K_{d}*\frac{\mathrm{d}&space;e(t)}{\mathrm{d}&space;x}" title="u(t)=K_{p}*e(t)+K_{d}*\frac{\mathrm{d} e(t)}{\mathrm{d} x}" />
  
  Derived of PD controller:

  <img src="https://latex.codecogs.com/svg.image?\frac{\mathrm{d}&space;u(t)}{\mathrm{d}&space;t}=K_{p}*\frac{\mathrm{d}&space;e(t)}{\mathrm{d}&space;t}&plus;K_{d}*\frac{\mathrm{d}^2e(t)&space;}{\mathrm{d}&space;t^2t" title="\frac{\mathrm{d} u(t)}{\mathrm{d} t}=K_{p}*\frac{\mathrm{d} e(t)}{\mathrm{d} t}+K_{d}*\frac{\mathrm{d}^2e(t) }{\mathrm{d} t^2t" />
  
  Convert to numerical:
  
  <img src="https://latex.codecogs.com/svg.image?\frac{u[k]-u[k-1]}{\Delta&space;t}=K_{p}*(\frac{e[k]-e[k-1]}{\Delta&space;t})&plus;K_{d}*(\frac{e[k]-2*e[k-1]&plus;e[k-2]}{\Delta&space;t^{2}})" title="\frac{u[k]-u[k-1]}{\Delta t}=K_{p}*(\frac{e[k]-e[k-1]}{\Delta t})+K_{d}*(\frac{e[k]-2*e[k-1]+e[k-2]}{\Delta t^{2}})" />
  
  Finding controller output:
  
  <img src="https://latex.codecogs.com/svg.image?u[k]=u[k-1]&plus;K_{p}*(e[k]-e[k-1])&plus;K_{d}*(\frac{e[k]-2*e[k-1]&plus;e[k-2]}{\Delta&space;t})" title="u[k]=u[k-1]+K_{p}*(e[k]-e[k-1])+K_{d}*(\frac{e[k]-2*e[k-1]+e[k-2]}{\Delta t})" />
  
  This the approximate which is implemented in the code:
  
  <img src="https://latex.codecogs.com/svg.image?u[k]\approx&space;u[k-1]&plus;K_{p}*(e[k]-e[k-1])&plus;K_{d}*(e[k]-2*e[k-1]&plus;e[k-2])" title="u[k]\approx u[k-1]+K_{p}*(e[k]-e[k-1])+K_{d}*(e[k]-2*e[k-1]+e[k-2])" />
  

## Bring Up the Turtlebot3

Launch on `SBC`:

    roslaunch turtlebot_bringup turtlebot3_robot.launch

## One Launch to run all the nodes

Intended to simplified the process after finish calibration. Launch one to run all the nodes on `Remote Pc`:

    $ roslaunch turtlebot3_lane_following one_ring_to_rule_them_all.launch


