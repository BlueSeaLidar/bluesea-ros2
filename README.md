# BLUESEA ROS2 driver #

## Overview ##
----------
BLUESEA ROS2 driver is specially designed to connect to the lidar products produced by our company. The driver can run on operating systems with ROS2 installed, and mainly supports ubuntu series operating systems (14.04LTS-20.04LTS). The hardware platforms that have been tested to run the ROS2 driver include: Intel x86 mainstream CPU platform, and some ARM64 hardware platforms (such as NVIDIA, Rockchip, Raspberry Pi, etc., which may need to update the cp210x driver).

## Get and build the BLUESEA ROS driver package ##
1.Get the BLUESEA ROS driver from Github and deploy the corresponding location

    mkdir bluesea2   											//create a folder and customize it
    cd bluesea2    												//into this folder
    git clone https://github.com/BlueSeaLidar/bluesea-ros2.git  src //download the driver package and rename it to src
2.Build

    catkin_make
3.Update the current ROS2 package environment

    source ./install/setup.sh


4.Using ROS2 launch to run drivers

	sudo chmod 777 /dev/ttyUSB0 (uart)			//  /dev/ttyUSB0  refers to the serial port name. If it is a serial/virtual serial port model, it needs to be authorized
    
    ros2 launch bluesea2 [launch file]    		//The specific launch file description is as follows

## Driver launch launch file  ##
explain：[launch file] refers to the configuration files in the src/launch folder, distinguished by functional categories

- uart_lidar.launch(clockwise/anticlockwise):			lidar with serial port connection method
- udp_lidar.launch(clockwise/anticlockwise):				lidar for UDP network communication
- vpc_lidar.launch(clockwise/anticlockwise)：				lidar with virtual serial port connection method
- dual_udp_lidar.launch(clockwise/anticlockwise)：		lidar with multiple UDP network communication(only one node)
- template：				All parameter definition templates


Main parameter configuration instructions：

    #ROS# (mandatory parameter for the framework)
    <param name="topic" value="scan" />#Publish topic
    <param name="frame_id" value="map" />#Name of the flag coordinate system
     #DATA# (driver-defined data level limiting parameter)
    <param name="min_dist" value="0.01" />#Minimum point cloud distance (m)
    <param name="max_dist" value="50.0"/>#Maximum point cloud distance (m)
    <param name="from_zero" value="false"/>#Whether start angle is from 0 (false is 180).
    <param name="output_scan" value="true" />#2D scan data (default)
    <param name="output_cloud" value="false"/>#3D spatial data.
    <param name="output_360" value="true" />#Output by frame.
    <param name="inverted" value="false"/>#Publish data angle parameter inverted(angle_min,angle_max,angle_increment).
    <param name="reversed" value="false"/>#Publish data point cloud data reversed (row from last point to first point)
    <param name="hard_resample" value="false"/>#hard_resample_factor(if lidar support this command)
    <param name="soft_resample" value="false"/>#Soft resample coefficient (need point cloud larger than the minimum number of points for soft resample)
    <param name="with_angle_filter" value="false"/>#Angle filter switch.
    <param name="min_angle" value="-3.1415926"/>#Minimum available angle.
    <param name="max_angle" value="3.1415926"/#Maximum available angle.
    <rosparam param="mask1" >[-3.14,3.14]</rosparam>#mask data for angle in this interval
    <param name="time_mode" value="0"/>#Timestamp source of packet (default 0 system time, 1 is lidar time synchronized time)
    <! -- <rosparam param="mask2" >[-1,0]</rosparam-->#Multiple segments to mask the angle of the interval, mask upwards +1
    #CUSTOM# (driver customization function)
    <param name="error_circle" value="3"/># Judge the weight of the point with distance 0 Three consecutive circles.
    <param name="error_scale" value="0.9"/># 90% of points with distance 0 in each circle will report error.
    <param name="group_listener" value="false" />#Only used to listen to multicast data (need to change the upload address of lidar by host computer, and fix the upload, then this driver will listen to the data).
    <param name="group_ip" value="224.0.0.11" />#Listen to multicast ip.
    #FITTER# (lidar different angular resolution parameters are different, need to be customized)
    <param name="filter_open" value="true"/>#Filter enable switch.
    <param name="max_range" value="20"/#Maximum range for filtering.
    <param name="min_range" value="0.5"/>#Minimum range for filtering.
    <param name="max_range_difference" value="0.1"/>#Physical range to judge the divergence.
    <param name="filter_window" value="1"/>#Range of subscripts for judging outliers
    #CONNECT# (parameter that drives the connection lidar)
    <param name="type" value="udp" />
    <param name="lidar_port" value="6543" />
    <param name="local_port" value="6668" />
    <param name="lidar_ip" value="192.168.158.98" />
    #GET# (the query command switch that the driver sends to the lidar)
    <param name="uuid" value="-1" /> #Query lidar SN number, -1 no query, >=0 query
    #SET# (set command switch that driver sends to lidar)
    <param name="rpm" value="-1"/#Set lidar rpm (different models of lidar can support different rpm, specific check the manual of the model):-1 not set 600 900 ... Setting
    <param name="sample_res" value="-1"/>#set angular resolution (different lidar models can support different angular resolution, check the manual of the model),-1 not set 0 original data 1 angular correction
    <param name="with_smooth" value="-1" />#Set de-smooth point, -1 not set 0 off 1 on.
    <param name="with_deshadow" value="-1" />#Set filtering, -1 not set 0 off 1 on
    <param name="alarm_msg" value="-1" />#set alarm message, -1 not set 0 off 1 on
    <param name="direction" value="-1"/>#Set direction of rotation(only used by lidar which support this command),-1 not set 0 off 1 on


## Driver Client Functional Description ##
source code locate at src/client.cpp
start/stop rotate：
    
    #rosrun bluesea2  bluesea2_client start  0  arg1 is (start/stop)  arg2 is lidar serial (Starting from 0, if it is a negative number, it means that all lidars are executed.)
    ros2 run bluesea2  bluesea_node_client start 
    ros2 run bluesea2 bluesea_node_client stop 
switch defense zones：
	
	#rosrun bluesea2  bluesea2_client switchZone  0    192.168.158.98     arg1 is switchZone   arg2 is defense zones to be switched  arg3 is lidar ip

set rpm：

	rosrun bluesea2  bluesea2_client rpm  0 600   arg1 is rpm   arg2 is lidar serial(start from zero)  arg3 is rpm to be set

## rosbag bag operating instructions ##

	rostopic list 
Get the topic list, the driver default topic name is /lidar1/scan

	rosbag record /lidar1/scan 

Start recording data.

The recorded file is named with a timestamp, to stop recording, CTRL+C in the current terminal 

	rosbag play packet name

Check the recorded packet in the path where the packet is stored, if it prompts failed connect master exception, then ros master first and then rosbag play.

## Business Support ##

Please contact the technical support (https://pacecat.com/) through the official website for specific usage problems.
