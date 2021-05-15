# ORB-SLAM2 with RealSense D435i

## Realsense d435i setup with ROS
1. install Realsense SDK from https://github.com/IntelRealSense/librealsense
2. install ROS Wrapper of Realsense from https://github.com/IntelRealSense/realsense-ros 
The realsense2\_camera package we got for running the Realsense camera is shown as the realsense2\_camera folder. We made some modifiction in some of the launch file 

launch Realsense D435i camera: 
```roslaunch realsense2_camera rs_rgbd.launch```


## ORB-SLAM2 Setup with Ubuntu 18.04 and ROS Melodic
This ORB-SLAM2 version in this repo is compatible with OpenCV 4 and solved several compilation error from the original ORB-SLAM2 repo. It also contains the yaml file we made for our Realsense D435i camera.

In order to build this repo, pull this ORB_SLAM2 repo and build following the instruction at the original ORB-SLAM2 repo

To run the ORB-SLAM2 with Realsense D435i camera:

```roslaunch realsense2_camera rs_rgbd.launch```
```rosrun ORB_SLAM2 RGBD ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/435i_ros.yaml /camera/depth_registered/image_raw:=/camera/depth/image_rect_raw /camera/rgb/image_raw:=/camera/color/image_raw```



