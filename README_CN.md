# 蓝海光电ROS2驱动程序V2.6(bluesea2_ros2_driver)

## 概述
----------
蓝海光电ROS2驱动程序是专门用于连接本公司生产的lidar产品。该驱动程序可以在安装了 ROS2 环境的操作系统中运行，主要支持ubuntu系列操作系统（18.04LTS-最新）。经测试可以运行该ROS驱动程序的硬件平台包括：intel x86 主流 cpu 平台，部分 ARM64 硬件平台（如 英伟达、瑞芯微，树莓派等，可能需要更新cp210x驱动）。

## 注意事项
请保证路径中不带中文字符，否则会编译失败

## 获取并构建蓝海ROS驱动包
1.从Github获取蓝海ROS2驱动程序,并部署对应位置

    mkdir bluesea2   											//创建一个文件夹，自定义即可
    cd bluesea2    												//进入该文件夹   
    git clone https://github.com/BlueSeaLidar/bluesea-ros2.git  src //下载驱动包并且重命名为src

2.构建

    colcon build
3.更新当前ROS2包环境

    source ./install/setup.sh


4.使用ROS2 launch运行驱动

	sudo chmod 777 /dev/ttyUSB0 (非网络款)			//这里的/dev/ttyUSB0指的是串口名称，如果是串口/虚拟串口款，需要赋权
    
    ros2 launch bluesea2 [launch file]    			//具体launch文件说明如下

## 驱动包launch配置文件说明
说明：[launch file]指的是src/launch文件夹下的配置文件，以功能类别区分

- uart_lidar.launch:			串口连接方式的激光雷达
- udp_lidar.launch:				udp网络通讯的激光雷达
- vpc_lidar.launch：				虚拟串口连接方式的激光雷达
- dual_udp_lidar.launch：		多个udp网络通讯的激光雷达
- template.launch：				全部参数定义模版
- LDS-U50C-S(only).launch:老款雷达，仅支持数据通讯
- LDS-U80C-S(only).launch:老款雷达，仅支持数据通讯
- 
主要参数配置说明：

    #ROS2#（框架必须的参数）
    <param name="scan_topic" value="scan"/>#发布scan话题
    <param name="cloud_topic" value="scan" />#发布cloud话题
    <param name="frame_id" value="map" />#标志坐标系的名称
     #DATA#（驱动自定义数据层面的限制参数）
    <param name="min_dist" value="0.01"/>#最小点云距离(m)
    <param name="max_dist" value="50.0"/>#最大点云距离(m)
    <param name="from_zero" value="false"/>#帧起始角度是否从0开始(false 为180)
    <param name="output_scan" value="true" />#二维扫描数据(默认)
    <param name="output_cloud" value="false"/>#三维空间数据
    <param name="output_cloud2" value="false"/>#三维空间数据 输出格式2
    <param name="output_360" value="true" />#按帧输出
	<param name="inverted" value="false"/>#发布数据角度参数反置(angle_min，angle_max,angle_increment)
    <param name="reversed" value="false"/>#发布数据点云数据倒转(从最后一个点排到第一个点)
    <param name="hard_resample" value="false"/>#硬采样系数(前提需要雷达支持该指令)
    <param name="soft_resample" value="false"/>#软采样系数(前提需要点云大于软采样的最小点数)
    <param name="with_angle_filter" value="false"/>#角度过滤开关
    <param name="min_angle" value="-3.1415926"/>#最小可用角度
    <param name="max_angle" value="3.1415926"/#最大可用角度
    <rosparam param="mask1" >[-3.14,3.14]</rosparam>#屏蔽该区间角度的数据
    <param name="time_mode" value="0"/>#数据包的时间戳来源(默认0系统时间，1是雷达时间同步的时间)
    <!-- <rosparam param="mask2" >[-1,0]</rosparam-->#多段屏蔽该区间角度的数据，mask往上+1
    #CUSTOM#（驱动自定义功能）
    <param name="error_circle" value="3"/>#判断距离为0的点的比重      连续三圈
    <param name="error_scale" value="0.9"/>#每圈距离为0的点占据90%  则报错
    <param name="group_listener" value="false" />#仅用于监听组播数据(需要上位机修改雷达的上传地址,并且固定上传，然后该驱动监听数据)
    <param name="group_ip" value="224.0.0.11" />#监听的组播ip
    #FITTER#(雷达不同角分辨率的参数不同，需定制)
    <param name="filter_open" value="true"/>#滤波生效开关
    <param name="max_range" value="20"/#滤波生效的最大距离
    <param name="min_range" value="0.5"/>#滤波生效的最小距离
    <param name="max_range_difference" value="0.1"/>#离异点判断的物理范围
    <param name="filter_window" value="1"/>#判断离异点下标的范围
    #CONNECT#（驱动连接雷达的参数）
    <param name="type" value="udp" />
    <param name="lidar_port" value="6543" />
    <param name="local_port" value="6668" />
    <param name="lidar_ip" value="192.168.158.98"/>
    #GET#（驱动发送给雷达的查询指令开关）
    <param name="uuid" value="-1" />#查询雷达SN号，-1不查询，>=0查询
    #SET#（驱动发送给雷达的设置指令开关）
    <param name="rpm" value="-1"/#设置雷达转速（不同型号雷达可支持的转速不相同，具体查看该型号的说明书）:-1不设置  600 900  ...设置
    <param name="resample_res" value="-1"/>#设置角分辨率（不同型号雷达可支持的角分辨率不相同，具体查看该型号的说明书）,-1不设置 0原始数据 1角度修正
    <param name="with_smooth" value="-1" />#设置去拖点,-1不设置 0关闭 1打开
    <param name="with_deshadow" value="-1" />#设置滤波,-1不设置 0关闭 1打开
    <param name="alarm_msg" value="-1"/>#设置报警信息,-1不设置 0关闭 1打开
    <param name="direction" value="-1"/>#设置旋转方向(仅支持该指令的雷达使用),-1不设置 0关闭 1打开
    #NTP#
    <param name="ntp_ip" value="192.168.0.111" /># NTP 服务器地址
    <param name="ntp_port" value="5678" />#NTP 服务器端口
    <param name="ntp_enable" value="1"/>#使能开关
## 驱动客户端功能说明
源码位于src/client.cpp
启停旋转：
    
    ros2 run bluesea2  bluesea2_client scan start      
    参数1 是话题   参数2是动作(start/stop)
切换防区：
	
	ros2 run bluesea2  bluesea2_client scan switchZone  0     
    参数1 是话题   参数2是动作(switchZone)  参数3是选择防区编号(0) 


设置转速：

	ros2 run bluesea2  bluesea2_client scan rpm  600 
    参数1 是话题   参数2是动作(set rpm)  参数3是设置rpm的值(600)

打开/关闭心跳包检测：

	ros2 run bluesea2  bluesea2_client heart check  1
   参数1是service name   参数2是动作(check)  参数3是是否打印(1/0)


## rosbag包操作说明

	ros2 topic list    获取话题列表，驱动默认的话题名称为 /lidar1/scan
	ros2 bag record  /lidar1/scan   开始录制数据

录制好的文件以时间戳来命名，要停止录制，在当前终端CTRL+C 

	ros2 bag play 数据包名称

在存放数据包的路径下查看录制的数据包，若提示failed connect master异常，则先ros master后在rosbag play

## 商务支持

具体使用问题请通过官网联系技术支持(https://pacecat.com/)
