# bluesea2
ROS2 driver for Lanhai USB/Network 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your workspace src folder
    2) Running `colcon build` to build 

How to run Lanhai ros node (Serial Port Version)
=====================================================================
1) Copy UDEV rule file : sudo cp src/LHLiDAR.rules /etc/udev/rules.d/
2) or Run : sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable


## if your lidar model is LDS-50C-2 :
* ros2 launch bluesea2 LDS-50C-2.py 

## if your lidar model is LDS-50C-C30E :
* ros2 launch bluesea2 LDS-50C-C30E.py 
    

3) optional : ros2 topic hz /scan
4) optional : rviz2 # 




