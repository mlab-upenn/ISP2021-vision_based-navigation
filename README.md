# ISP2021-vision_based-navigation
This is the github project for the F1Tenth Independent Study Projects 2021. In this project we are focusing on the development of different approaches to achieve a vision-based navigation for the F1TENTH car.

This branch consists the code for detecting lane lines using monocular camera and color based masking, with OpenCV.


## Requirements
- Linux Ubuntu (tested on versions 16.04)
- ROS Kinetic

## Installation
Installationa and run requirements from https://github.com/f1tenth/f1tenth_system



## Running the code
* `Step 1:` Launch 'teleop.launch' file from racecar package using roslaunch.
* `Step 2:` Start monocular camera using cv_camera ROS node.
rosparam set cv_camera/device_id 0 (check device id for connected usb camera)
rosrun cv_camera cv_camera_node
* `Step 3:` Run node 'im_process.py' to detect lane line markings and give steering output using rosrun.



## Folder Structure

All main scripts depend on the following subfolders:

1. To find 'teleop.launch' file:  ISP2021-vision_based-navigation/src/f110_system/racecar/racecar/launch/teleop.launch 
2. To find 'im_process.py' file: ISP2021-vision_based-navigation/src/f110_system/racecar/racecar/scripts/im_process.py
