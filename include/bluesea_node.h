#include"parser.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_srvs/srv/empty.hpp>


struct PubHub
{
	pthread_mutex_t mtx;
	int nfan;
	RawData *fans[MAX_FANS];
	int error_num;
	int offsetangle;
	int offsetidx;
	std::vector<DataPoint>consume;//统计完剩余的点数
	uint32_t ts_beg[2];
	uint32_t ts_end[2];
};

struct ConnectArg
{
	std::string laser_topics;
	std::string cloud_topics;
	std::string arg1;
	int arg2;
};
struct Range
{
	double min;
	double max;
};
//定制隐藏功能
struct Custom
{
	int error_circle;
	double error_scale;
	bool is_group_listener;
	std::string group_ip;
};
//过滤算法
struct Fitter
{
	bool isopen;
	int type;
	double max_range;
	double min_range;
	double max_range_difference;
	int filter_window;

};
struct ArgData
{
	std:: string frame_id;
	int dev_id;
	std::vector<ConnectArg>connectargs;
	int raw_bytes;
	bool from_zero;
	bool output_scan;
	bool output_cloud;
	bool inverted;
	bool reversed;
	bool hard_resample;
	bool soft_resample;
	bool with_angle_filter;
	double min_angle;
	double max_angle;
	bool output_360;
	double min_dist;
	double max_dist;
	std::vector<Range>masks;
	std::string type;
	int localport;
	int num;
	int uuid;
	int rpm;
	int model;
	double resample;
	int with_smooth;
	int with_deshadow;
	int alarm_msg;
	int direction;
	int unit_is_mm;
	int with_confidence;
	int ats;
	Fitter fitter;
	Custom custom;

};
//中断进程
void closeSignal(int sig);
//指定编号雷达发送指令
bool SendCmd(int len, char *cmd, int index);
//获取一帧第一个点所在的扇区，以及该点所在扇区的位置
bool autoGetFirstAngle2(HPublish pub, bool from_zero);
//对读取的数据进行解析    参数:实际获取的数据    配置文件对应的参数
int GetAllFans(HPublish pub,ArgData argdata,uint8_t &counterclockwise);
//对一帧的数据进行处于并上传  点云
void PublishLaserScan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub, HPublish pub, ArgData argdata,uint8_t counterclockwise);
//对一个扇区的数据处理并上传
void PublishLaserScanFan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub, RawData *fan,std::string &frame_id,double min_dist, double max_dist,uint8_t inverted,uint8_t reversed);
//对一帧的数据进行处于并上传  三维
//void PublishCloud(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &cloud_pub, HPublish pub, ArgData argdata);