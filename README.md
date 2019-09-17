# Pickpack

## 1. Overview

This is a ROS package presents a perception-to-manipulation system for picking thin objects from clutter. 
The manipulation consists of four steps: detecting the object of instance, approaching the target overhead, descending till the fingertip contacts the object, tilting to adjust the final grasp pose.
Object detection is implemented on Mask R-CNN, a deep neural network for instance segmentation, while descending and tilting are implemented with tactile sensors.

<p align = "center">
<img src="files/openlid.gif" width="360" height="202"> <img src="files/carton.gif" width="360" height="202"> 
<img src="files/acrylic_2finger.gif" width="360" height="202"> <img src="files/acrylic_3finger.gif" width="360" height="202">
</p>

## 2. Prerequisites

### 2.1 Hardware
1. [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
2. [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
3. [**Robotiq FT300**](https://robotiq.com/products/ft-300-force-torque-sensor) Force Torque Sensor
4. [**Realsense SR300**](https://www.intelrealsense.com/coded-light/)

### 2.2 Software
1. Our package was developed in [**Ubuntu 16.04**](http://releases.ubuntu.com/16.04/) and [**ROS Kinetic**](http://wiki.ros.org/ROS/Installation).
2. [**urx**](https://github.com/ThomasTimm/ur_modern_driver/):Python library to control UR10 robot.
3. [Robotiq ROS package](http://wiki.ros.org/robotiq/): ROS driver for Robotiq adaptive gripper and force torque sensor.
4. [Realsense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros): ROS wrapper for Realsense SR300.
5. [AprilTag ROS Wrapper](https://github.com/AprilRobotics/apriltag_ros): ROS wrapper for apriltag library.
6. [Mask R-CNN](https://github.com/matterport/Mask_RCNN): Framework for Object Detection and Segmentation

## 3. Build on ROS
In your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/oliviaHKUST/pickpack.git
cd ..
catkin_make
```

## 4. Get started
The following instructions will help you build up the software step by step.

1. Follow the tutorial in [Universal Robot package for ROS Kinetic](http://wiki.ros.org/universal_robot) and [Robotiq ROS package](http://wiki.ros.org/robotiq/) to set up hardware properly.
2. Run Realsense SR300 camera in ROS. See [link](http://wiki.ros.org/RealSense).
3. Connect Arduino and tactile sensor, output the sensor readings in ROS. See [link](http://wiki.ros.org/rosserial_arduino/Tutorials).
4. Setup frames:
   ```
   cd scripts
   ```
   ```
   python frame_transform.py 
   ```
5. Open a terminal, run object detection:
   ```
   cd samples
   ```
   ```
   jupyter notebook
   ```
   Open ```instance_segmentation.ipynb```. For loading ```BLISTER_MODEL_PATH```, please refer to [here](https://hkustconnect-my.sharepoint.com/:u:/g/personal/ztong_connect_ust_hk/EQ_7Mi8_-pBCrZwDXYc21QIBEgfSpwU2K-eZ1M3d01JVcQ?e=dr3e5G).
   
6. Open another terminal, run manipulation:
   ```
   cd scripts
   ```
   ```
   jupyter notebook
   ```
   Open ```thin_object_bin_pick_mani.ipynb```

## Author
Qianyi Xu(qxuaj@connect.ust.hk), Zhekai Tong (ztong@connect.ust.hk) and Tierui He (theae@connect.ust.hk)
