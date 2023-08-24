# bluesea2
ROS2 driver for Lanhai USB/Network 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your workspace src folder
    2) cd  ..
    3) Running `colcon build` to build  (The following commands are executed in this directory)

How to run Lanhai ros node (Serial Port Version)
=====================================================================
1) Copy UDEV rule file : sudo cp src/LHLiDAR.rules /etc/udev/rules.d/
2) or Run : sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable


## if your lidar model is LDS-50C-2 :
* ros2 launch bluesea2 LDS-50C-2.py 

## if your lidar model is LDS-50C-C30E :
* ros2 launch bluesea2 LDS-50C-C30E.py 
    
## if your lidar model is LDS-50C-C20E :
* ros2 launch bluesea2 LDS-50C-C20E.py 

## if your lidar model is LSS-40D-C20E :
* ros2 launch bluesea2 LDS-40D-C20E.py 

    
3) optional : ros2 topic hz /scan

4) optional : rviz2 # 


How to control Lanhai ros node  start  and stop
=====================================================================
* client:   
   
 		ros2 run bluesea2  bluesea_node_client start 
  
 		ros2 run bluesea2 bluesea_node_client stop 
 
	           
* server:  
    
		ros2 launch  bluesea2  xxxx.launch 


