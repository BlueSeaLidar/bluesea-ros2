#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <pthread.h>

#include <sys/time.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_srvs/srv/empty.hpp>

//#include "timer.h"
#include "reader.h"
#include <cmath>

#define ROS2Verision "1.4.6"

HReader g_reader = NULL;
std::string g_type = "uart";

struct Range
{
	double min;
	double max;
};
struct PubHub
{
	pthread_mutex_t mtx;
	int nfan;
	RawData *fans[MAX_FANS];
};
void closeSignal(int)
{
	rclcpp::shutdown();
	exit(1);
}

bool SendCmd(int len, char *cmd)
{
	if (g_type == "uart")
	{
		return SendUartCmd(g_reader, len, cmd);
	}
	else if (g_type == "udp")
	{
		int tmp = 0;
		memcpy(&tmp, g_reader, sizeof(int));
		for (int i = 0; i < tmp; i++)
		{
			SendUdpCmd(g_reader, i, len, cmd);
		}
		return true;
	}
	else if (g_type == "tcp")
	{
		return SendTcpCmd(g_reader, len, cmd);
	}
	return false;
}

void PublishData(HPublish pub, int n, RawData **fans)
{
	int skip = 0;
	RawData *drop[MAX_FANS];
	PubHub *hub = (PubHub *)pub;

	pthread_mutex_lock(&hub->mtx);

	if (hub->nfan + n > MAX_FANS)
	{
		int nr = hub->nfan + n - MAX_FANS;

		for (int i = 0; i < nr; i++)
			drop[skip++] = hub->fans[i];

		for (int i = nr; i < hub->nfan; i++)
		{
			hub->fans[i - nr] = hub->fans[i];
		}

		hub->nfan -= nr;
	}

	for (int i = 0; i < n; i++)
	{
		hub->fans[hub->nfan++] = fans[i];
	}

	pthread_mutex_unlock(&hub->mtx);

	if (skip > 0)
		printf("drop %d fans\n", skip);
	for (int i = 0; i < skip; i++)
	{
		delete drop[i];
	}
}

#if 0
int angle_cmp(const void* p1, const void* p2)
{
	const RawData** pp1 = (const RawData**)p1;
	const RawData** pp2 = (const RawData**)p2;
	return (*pp1)->ros_angle - (*pp2)->ros_angle;
}
#endif

void resample(RawData *dat, int NN)
{
	int *index = new int[NN];
	double *errs = new double[NN];

	for (int i = 0; i < NN; i++)
		index[i] = -1;

	for (int i = 0; i < dat->N; i++)
	{
		if (dat->points[i].distance == 0)
			continue;

		int idx = round(double(i * NN) / dat->N);
		if (idx < NN)
		{
			double err = fabs(double(dat->span * i) / dat->N - double(dat->span * idx) / NN);
			if (index[idx] == -1 || err < errs[idx])
			{
				index[idx] = i;
				errs[idx] = err;
			}
		}
	}

	for (int i = 1; i < NN; i++)
	{
		if (index[i] != -1)
		{
			dat->points[i] = dat->points[index[i]];
		}
		else
		{
			dat->points[i].distance = 0;
			dat->points[i].confidence = 0;
		}
	}

	dat->N = NN;

	delete index;
	delete errs;
}

bool GetFan(HPublish pub, bool with_resample, double resample_res, RawData **fans)
{
	bool got = false;
	PubHub *hub = (PubHub *)pub;

	pthread_mutex_lock(&hub->mtx);

	if (hub->nfan > 0)
	{
		fans[0] = hub->fans[0];
		for (int i = 1; i < hub->nfan; i++)
			hub->fans[i - 1] = hub->fans[i];
		hub->nfan--;
		got = true;
	}

	pthread_mutex_unlock(&hub->mtx);

	if (with_resample) // && resample_res > 0.05)
	{
		int NN = fans[0]->span / (10 * resample_res);
		if (NN < fans[0]->N)
			resample(fans[0], NN);
	}

	return got;
}

int GetAllFans(HPublish pub, bool with_resample, double resample_res, RawData **fans, bool from_zero,int collect_angle,
			   uint32_t *ts_beg, uint32_t *ts_end)
{
	PubHub *hub = (PubHub *)pub;
	// RawData* drop[MAX_FANS];
	pthread_mutex_lock(&hub->mtx);
	int cnt = 0;
	for (int i = 1; i < hub->nfan; i++)
	{
		if ((from_zero && hub->fans[i]->angle == 0) ||
			(!from_zero && hub->fans[i]->angle == 1800+collect_angle*10))
		{
			ts_end[0] = hub->fans[i]->ts[0];
			ts_end[1] = hub->fans[i]->ts[1];
			cnt = i;
			break;
		}
	}

	if (cnt > 0)
	{
		for (int i = 0; i < cnt; i++)
		{
			fans[i] = hub->fans[i];
		}
		for (int i = cnt; i < hub->nfan; i++)
			hub->fans[i - cnt] = hub->fans[i];
		hub->nfan -= cnt;
	}
	pthread_mutex_unlock(&hub->mtx);

	if (cnt > 0)
	{
		bool circle = true;
		int total = fans[0]->span;
		ts_beg[0] = fans[0]->ts[0];
		ts_beg[1] = fans[0]->ts[1];
		// printf("ts = %d.%d\n",fans[0]->ts[0],fans[0]->ts[1]);
		for (int i = 0; i < cnt - 1; i++)
		{
			if ((fans[i]->angle + fans[i]->span)%3600 != fans[i + 1]->angle &&
				fans[i]->angle + fans[i]->span != 3600)
				circle = false;
			total += fans[i + 1]->span;
		}
		if (!circle || total != 3600)
		{
			printf("%d drop %d fans\n", total, cnt);
			// clean imcomplent datas
			for (int i = 0; i < cnt; i++)
				delete fans[i];
			cnt = 0;
		}
	}
	if (cnt > 0)
	{
		if (with_resample)
		{
			for (int i = 0; i < cnt; i++)
			{
				int NN = fans[i]->span / (10 * resample_res);
				if (NN < fans[i]->N)
				{
					resample(fans[i], NN);
				}
				else if (NN > fans[i]->N)
				{
					printf("fan [%d] %d less than %d\n", i, fans[i]->N, NN);
				}
			}
		}
	}

	return cnt;
}

void SetTimeStamp(RawData *dat)
{
	timeval t;
	gettimeofday(&t, NULL);
	// ros::Time t = ros::Time::now();
	dat->ts[0] = t.tv_sec;
	dat->ts[1] = t.tv_usec * 1000;
}

double ROSAng(double ang)
{
	ang = -ang;
	return ang < -180 ? ang + 360 : ang;
	// return ang < 180 ? ang : ang - 360;
}

int GetCount(int nfan, RawData **fans, double min_deg, double max_deg, double &min_pos, double &max_pos)
{
	int N = 0, cnt = 0;

	for (int j = 0; j < nfan; j++)
	{
		for (int i = fans[j]->N - 1; i >= 0; i--, cnt++)
		{
			double deg = ROSAng(fans[j]->points[i].degree);
			if (deg < min_deg || deg > max_deg)
				continue;
			if (N == 0)
			{
				min_pos = deg;
				max_pos = deg;
			}
			else
			{
				if (min_pos > deg)
					min_pos = deg;
				if (max_pos < deg)
					max_pos = deg;
			}
			N++;
		}
	}
	// printf("angle filter [%f, %f] %d to %d, [%f, %f]\n", min_deg, max_deg, cnt, N, min_pos, max_pos);
	return N;
}


void PublishLaserScanFan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub,RawData *fan,
						 std::string &frame_id,
						 double min_dist, double max_dist,
						 bool with_filter, double min_ang, double max_ang,
						 bool inverted, bool reversed, double zero_shift,
						 const std::vector<Range> &custom_masks, int collect_angle)
{
	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;
	int N = fan->N;
	
	if(zero_shift>M_PI||zero_shift<-M_PI)
	{
		int num = zero_shift/M_PI;
		zero_shift=zero_shift-num*M_PI;
	}	
	if(zero_shift!=0)
	{
		double zero_shift_tmp=zero_shift* 180 / M_PI;
		for (int i = 0; i <N; i++)
		{
			double deg =fan->points[i].degree+zero_shift_tmp;
			fan->points[i].degree=deg;
		}
	
	}
	if(reversed)
		collect_angle = -collect_angle;
	double min_pos, max_pos;
	if (with_filter)
	{
		N = GetCount(1, &fan, min_deg, max_deg, min_pos, max_pos);
		if (N < 2)
			return;
		min_pos = min_pos * M_PI / 180;
		max_pos = max_pos * M_PI / 180;
	}
	else
	{
		if (inverted)
		{
			min_pos = ROSAng(fan->angle / 10+ collect_angle) * 10 * M_PI / 1800;
			max_pos = -min_pos + (fan->span+collect_angle*10 )* M_PI / 1800;
		}
		else
		{

			min_pos = ROSAng(fan->angle / 10- collect_angle) * 10 * M_PI / 1800;
			max_pos = min_pos - (fan->span- collect_angle*10) * M_PI / 1800;
		}
	}

	sensor_msgs::msg::LaserScan msg;

	// msg.header.stamp = ros::Time::now();
	msg.header.stamp.sec = fan->ts[0];
	msg.header.stamp.nanosec = fan->ts[1];

	msg.header.frame_id = frame_id;

	double scan_time = 1 / 100.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / fan->N;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;

	msg.intensities.resize(N); // fan->N);
	msg.ranges.resize(N);	   // fan->N);

	if (inverted)
	{
		msg.angle_min = min_pos;
		msg.angle_max = max_pos;
		msg.angle_increment = (fan->span * M_PI / 1800) / fan->N;
	}
	else
	{
		msg.angle_min = max_pos;
		msg.angle_max = min_pos;
		msg.angle_increment = -(fan->span * M_PI / 1800) / fan->N;
	}

	//msg.angle_min += zero_shift;
	//msg.angle_max += zero_shift;

	N = 0;
	if (reversed)
	{
		for (int i = fan->N - 1; i >= 0; i--)
		{
			double deg = ROSAng(fan->points[i].degree);
			if (with_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			// customize angle filter
			bool custom = false;
			for (unsigned int k = 0; k < custom_masks.size() && !custom; k++)
			{
				if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
					custom = true;
			}

			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
				msg.ranges[N] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[N] = d;

			msg.intensities[N] = fan->points[i].confidence;
			N++;
		}
	}
	else
	{
		for (int i = 0; i < fan->N; i++)
		{
			double deg = ROSAng(fan->points[i].degree);
			if (with_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			// customize angle filter
			bool custom = false;
			for (unsigned int k = 0; k < custom_masks.size() && !custom; k++)
			{
				if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
					custom = true;
			}

			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
				msg.ranges[N] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[N] = d;

			msg.intensities[N] = fan->points[i].confidence;
			N++;
		}
	}
	laser_pub->publish(msg); 
}
#if 0
void PublishCloud(ros::Publisher& cloud_pub, int nfan, RawData** fans, std::string& frame_id, 
		double max_dist,
		bool with_filter, double min_ang, double max_ang)
{
	sensor_msgs::PointCloud cloud; 
	//cloud.header.stamp = ros::Time::now();

	cloud.header.stamp.sec = fans[0]->ts[0];
	cloud.header.stamp.nsec = fans[0]->ts[1];

	cloud.header.frame_id = frame_id; 

	int N = 0;
	for (int i=0; i<nfan; i++) N += fans[i]->N;

	cloud.points.resize(N);
	cloud.channels.resize(1); 
	cloud.channels[0].name = "intensities"; 
	cloud.channels[0].values.resize(N);


	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;
	if (with_filter) 
	{
		N = GetCount(nfan, fans, min_deg, max_deg);
	}

	int idx = 0;
	for (int j=0; j<nfan; j++) 
	{
		for (int i=0; i<fans[j]->N; i++) 
		{
			if (with_filter) {
				if (ROSAng(fans[j]->points[i].degree) < min_deg) continue;
				if (ROSAng(fans[j]->points[i].degree) > max_deg) continue;
			}

			float r = fans[j]->points[i].distance/1000.0 ; 
			// float a = j*M_PI/5 + i*M_PI/5/dat360[j].N;
			float a = -fans[j]->points[i].degree * M_PI/180;

			cloud.points[idx].x = cos(a) * r;
			cloud.points[idx].y = sin(a) * r;
			cloud.points[idx].z = 0;
			cloud.channels[0].values[idx] = fans[j]->points[i].confidence;
			idx++;
		}
	} 
	cloud_pub.publish(cloud);
}

#endif

int time_cmp(const uint32_t *t1, const uint32_t *t2)
{
	if (t1[0] > t2[0])
		return 1;
	else if (t1[0] < t2[0])
		return -1;
	else if (t1[1] > t2[1])
		return 1;
	else if (t1[1] < t2[1])
		return -1;

	return 0;
}

void PublishLaserScan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub,
					  int nfan, RawData **fans, std::string &frame_id,
					  double min_dist, double max_dist, bool with_filter, double min_ang, double max_ang,
					  bool inverted, bool reversed, double zero_shift,
					  bool from_zero, uint32_t *ts_beg, uint32_t *ts_end,
					  const std::vector<Range> &custom_masks, int collect_angle)
{
	sensor_msgs::msg::LaserScan msg;

	int N = 0;
	if(zero_shift>M_PI||zero_shift<-M_PI)
	{
		int num = zero_shift/M_PI;
		zero_shift=zero_shift-num*M_PI;
	}
		
	double zero_shift_tmp=zero_shift* 180 / M_PI;
	for (int j = 0; j < nfan; j++)
	{
		N += fans[j]->N;
		if(zero_shift==0)
			continue;
		for (int i = 0; i < fans[j]->N; i++)
		{
			double deg =fans[j]->points[i].degree+zero_shift_tmp;
			fans[j]->points[i].degree=deg;
		}
	}

	msg.header.stamp.sec = ts_beg[0];
	msg.header.stamp.nanosec = ts_beg[1];

	double ti = double(ts_beg[0]) + double(ts_beg[1]) / 1000000000.0;
	double tx = double(ts_end[0]) + double(ts_end[1]) / 1000000000.0;

	msg.scan_time = tx - ti; // nfan/(nfan-1);
	msg.time_increment = msg.scan_time / N;

	msg.header.frame_id = frame_id;

	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;

	//min_deg -= zero_shift* 180 / M_PI;
	//max_deg -= zero_shift* 180 / M_PI;
	if(reversed)
		collect_angle = -collect_angle;
	if (with_filter)
	{
		double min_pos, max_pos;
		N = GetCount(nfan, fans, min_deg, max_deg, min_pos, max_pos);
		if (inverted)
		{
			msg.angle_min = min_pos * M_PI / 180+ (float)collect_angle / 180 * M_PI;
			msg.angle_max = max_pos * M_PI / 180+ (float)collect_angle / 180 * M_PI;
			//printf("data1:deg:%lf   %lf  angle:%lf   %lf   %d\n",min_deg,max_deg,msg.angle_min,msg.angle_max,N);
		}
		else
		{
			msg.angle_min = max_pos * M_PI / 180-(float)collect_angle / 180 * M_PI;
			msg.angle_max = min_pos * M_PI / 180- (float)collect_angle / 180 * M_PI;
		}
		msg.angle_increment = (msg.angle_max - msg.angle_min) / (N - 1);
	}
	else
	{
		if (from_zero)
		{
			if (inverted)
			{
				msg.angle_min = 0;
				msg.angle_max = 2 * M_PI * (N - 1) / N;
				msg.angle_increment = M_PI * 2 / N;
			}
			else
			{
				msg.angle_min = 2 * M_PI * (N - 1) / N;
				msg.angle_max = 0;
				msg.angle_increment = -M_PI * 2 / N;
			}
		}
		else
		{
			if (inverted)
			{
				msg.angle_min = -M_PI + (float)collect_angle / 180 * M_PI;
				msg.angle_max = M_PI - (2 * M_PI) / N + (float)collect_angle / 180 * M_PI;
				msg.angle_increment = M_PI * 2 / N;
			}
			else
			{
				msg.angle_min = M_PI - (float)collect_angle / 180 * M_PI;
				msg.angle_max = -M_PI + (2 * M_PI) / N - (float)collect_angle / 180 * M_PI;
				msg.angle_increment = -M_PI * 2 / N;
			}
		}
	}

	//msg.angle_min += zero_shift;
	//msg.angle_max += zero_shift;
	//printf("angle:%lf   %lf   %d\n",msg.angle_min,msg.angle_max,N);

	if (fans[0]->counterclockwise != 0)
	{
		double d = msg.angle_min;
		msg.angle_min = msg.angle_max;
		msg.angle_max = d;
		msg.angle_increment = -msg.angle_increment;
		msg.angle_min -= msg.angle_increment;
		msg.angle_max -= msg.angle_increment;
	}

	msg.intensities.resize(N);
	msg.ranges.resize(N);
	
	N = 0;
	if (reversed)
	{
		for (int j = nfan - 1; j >= 0; j--)
		{
			for (int i = fans[j]->N - 1; i >= 0; i--)
			{
				double deg = ROSAng(fans[j]->points[i].degree);
				if (with_filter)
				{
					if (deg < min_deg)
						continue;
					if (deg > max_deg)
						continue;
				}

				double d = fans[j]->points[i].distance / 1000.0;

				// customize angle filter
				bool custom = false;
				for (unsigned int k = 0; k < custom_masks.size() && !custom; k++)
				{
					if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
						custom = true;
				}

				if (fans[j]->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
					msg.ranges[N] = std::numeric_limits<float>::infinity();
				else
					msg.ranges[N] = d;

				msg.intensities[N] = fans[j]->points[i].confidence;
				N++;
			}
		}
	}
	else
	{
		for (int j = 0; j < nfan; j++)
		{
			for (int i = 0; i < fans[j]->N; i++)
			{
				double deg = ROSAng(fans[j]->points[i].degree);
				
				if (with_filter)
				{
					if (deg < min_deg)
						continue;
					if (deg > max_deg)
						continue;
				}
				// customize angle filter
				bool custom = false;
				for (unsigned int k = 0; k < custom_masks.size() && !custom; k++)
				{
					if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
						custom = true;
				}

				double d = fans[j]->points[i].distance / 1000.0;
				if (fans[j]->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
					msg.ranges[N] = std::numeric_limits<float>::infinity();
				else
					msg.ranges[N] = d;

				
				msg.intensities[N] = fans[j]->points[i].confidence;
				N++;
			}
		}
	}

	laser_pub->publish(msg);
}

#if 0
void setup_params(bluesea2::DynParamsConfig &config, uint32_t level) 
{
	ROS_INFO("Change RPM to [%d]", config.rpm);

	char cmd[32];
	sprintf(cmd, "LSRPM:%dH", config.rpm);

	SendCmd(strlen(cmd), cmd);
}
#endif

bool should_start = true;
// service call back function
bool stop_motor(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{

	should_start = false;
	// ROS_INFO("Stop motor");
	char cmd[] = "LSTOPH";
	return SendCmd(6, cmd);
}

// service call back function
bool start_motor(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
{
	should_start = true;
	char cmd[] = "LSTARH";

	// ROS_INFO("Stop motor");

	return SendCmd(6, cmd);
}

uint32_t get_device_ability(const std::string &platform)
{
	if (platform == "LDS-50C-E")
	{
		return // DF_UNIT_IS_MM | DF_WITH_INTENSITY |
			DF_DESHADOWED |
			DF_SMOOTHED |
			DF_WITH_RESAMPLE |
			DF_WITH_UUID |
			EF_ENABLE_ALARM_MSG|
			DF_WITH_RPM;
	}
	else if (platform == "LDS-50C-2")
	{
		return DF_UNIT_IS_MM |
			   DF_WITH_INTENSITY |
			   DF_DESHADOWED |
			   DF_SMOOTHED |
			   DF_WITH_UUID|
			   DF_WITH_RPM;;
	}
	else if (platform == "LDS-50C-S")
	{
		return DF_UNIT_IS_MM | DF_WITH_INTENSITY;
	}

	return 0;
}

void split(const std::string &s, char delim, int *elems)
{
	int idx = 0;
	std::stringstream ss(s);
	std::string number;

	while (std::getline(ss, number, delim))
	{
		elems[idx++] = atoi(number.c_str());
		elems[idx] = 0;
	}
}

#define READ_PARAM(TYPE, NAME, VAR, INIT)     \
	TYPE VAR = INIT;                          \
	node->declare_parameter<TYPE>(NAME, VAR); \
	node->get_parameter(NAME, VAR);

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("bluesea_node");

	signal(SIGINT, closeSignal);
	READ_PARAM(std::string, "type", type, "uart");
	g_type = type;

	READ_PARAM(std::string, "platform", platform, "LDS-50C-S");

	// for serial port comm
	READ_PARAM(std::string, "port", port, "/dev/ttyUSB0");
	READ_PARAM(int, "baud_rate", baud_rate, 500000);

	READ_PARAM(std::string, "rate_list", rate_list, "230400,256000,500000,768000,1000000");
	int rates[100];
	split(rate_list, ',', rates);

	READ_PARAM(bool, "group_listener", is_group_listener, false);

	int lidar_ports[MAX_LIDARS];
	std::string lidar_ips[MAX_LIDARS];
	std::string laser_topics[MAX_LIDARS];
	std::string cloud_topics[MAX_LIDARS];
	// for network comm
	lidar_ips[0] = "192.168.158.91";
	node->declare_parameter<std::string>("lidar_ip", lidar_ips[0]);
	node->get_parameter("lidar_ip", lidar_ips[0]);

	lidar_ports[0] = 6543;
	node->declare_parameter<int>("lidar_port", lidar_ports[0]);
	node->get_parameter("lidar_port", lidar_ports[0]);

	// READ_PARAM(std::string, "lidar_ip", lidar_ips[0], "192.168.158.91");
	READ_PARAM(std::string, "group_ip", group_ip, "224.1.1.91");
	// READ_PARAM(int, "lidar_port", lidar_ports[0], 5000);
	READ_PARAM(int, "local_port", local_port, 50122);
	// topic
	laser_topics[0] = "scan";
	node->declare_parameter<std::string>("topic", laser_topics[0]);
	node->get_parameter("topic", laser_topics[0]);
	// READ_PARAM(std::string, "topic", topics[0], "scan");

	// cloud_topic
	cloud_topics[0] = "scan";
	node->declare_parameter<std::string>("cloud_topic", cloud_topics[0]);
	node->get_parameter("cloud_topic", cloud_topics[0]);

	int lidar_count = 1;
	for (int i = 1; i < MAX_LIDARS; i++)
	{
		char s[32], t[32];

		sprintf(s, "lidar%d_ip", i);
		lidar_ips[i] = "";
		node->declare_parameter<std::string>(s, lidar_ips[i]);
		node->get_parameter(s, lidar_ips[i]);
		// READ_PARAM(std::string, s, lidar_ips[i], std::string(""));
		if (lidar_ips[i].length() == 0)
			break;

		sprintf(s, "lidar%d_port", i);
		lidar_ports[i] = 0;
		node->declare_parameter<int>(s, lidar_ports[i]);
		node->get_parameter(s, lidar_ports[i]);
		// READ_PARAM(int, s, lidar_ports[i], 0);
		// priv_nh.param(s, lidar_ports[i], 0);
		if (lidar_ports[i] <= 0)
			break;

		sprintf(s, "topic%d", i);
		sprintf(t, "scan%d", i);
		laser_topics[i] = t;
		node->declare_parameter<std::string>(s, laser_topics[i]);
		node->get_parameter(s, laser_topics[i]);

		sprintf(s, "cloud_topic%d", i);
		sprintf(t, "cloud%d", i);
		cloud_topics[i] = t;
		node->declare_parameter<std::string>(s, cloud_topics[i]);
		node->get_parameter(s, cloud_topics[i]);
		// READ_PARAM(std::string, s, topics[i], std::string(t));
		lidar_count++;
	}
	printf("we will connect to %d lidars\n", lidar_count);
	for (int i = 0; i < lidar_count; i++)
	{
		printf("lidar[%d] address %s:%d, topic `%s\n", i,
			   lidar_ips[i].c_str(), lidar_ports[i],
			   laser_topics[i].c_str());
	}
	// device identity in data packets, used when multiple lidars connect to single controller
	READ_PARAM(int, "dev_id", dev_id, ANYONE); //

	// raw data format
	// READ_PARAM(int, "normal_size", normal_size, -1); // -1 : allow all packet, N : drop packets whose points less than N
	READ_PARAM(int, "raw_bytes", raw_bytes, 3);					// packet mode : 2bytes or 3bytes
	READ_PARAM(bool, "unit_is_mm", unit_is_mm, true);			// 0 : distance is CM, 1: MM
	READ_PARAM(bool, "with_confidence", with_confidence, true); //
	READ_PARAM(bool, "with_checksum", with_chk, true);			// true : enable packet checksum

	// is lidar inverted
	READ_PARAM(bool, "inverted", inverted, false);
	READ_PARAM(bool, "reversed", reversed, false);

	READ_PARAM(bool, "with_smooth", with_smooth, true);		// lidar data smooth filter
	READ_PARAM(bool, "with_deshadow", with_deshadow, true); // data shadow filter

	// angle composate
	READ_PARAM(bool, "hard_resample", hard_resample, true);		 // resample angle resolution
	READ_PARAM(bool, "soft_resample", with_soft_resample, true); // resample angle resolution
	READ_PARAM(double, "resample_res", resample_res, 0.5);		 // resample angle resolution @ 0.5 degree
	if (resample_res < 0.05 || resample_res > 1)
	{
		with_soft_resample = false;
		hard_resample = false;
	}

	// data output
	READ_PARAM(bool, "output_scan", output_scan, true);	   // true: enable output angle+distance mode, 0: disable
	READ_PARAM(bool, "output_cloud", output_cloud, false); // false: enable output xyz format, 0 : disable
	READ_PARAM(bool, "output_360", output_360, true);	   // true: packet data of 360 degree (multiple RawData), publish once
	// false: publish every RawData (36 degree)
	// RPM
	READ_PARAM(int, "rpm", init_rpm, -1); // set motor RPM

	// angle filter
	READ_PARAM(bool, "with_angle_filter", with_angle_filter, false); // true: enable angle filter, false: disable
	READ_PARAM(double, "min_angle", min_angle, -M_PI);				 // angle filter's low threshold, default value: -pi
	READ_PARAM(double, "max_angle", max_angle, M_PI);				 // angle filters' up threashold, default value: pi

	// range limitation
	READ_PARAM(double, "max_dist", max_dist, 9999.0); // max detection range, default value: 9999M
	READ_PARAM(double, "min_dist", min_dist, 0.0);

	// frame information
	READ_PARAM(std::string, "frame_id", frame_id, "LH_laser"); // could be used for rviz
															   // READ_PARAM(std::string, "firmware_version", firmware_number, 2);

	// zero position
	READ_PARAM(double, "zero_shift", zero_shift, 0.0);

	READ_PARAM(bool, "from_zero", from_zero, false);
	READ_PARAM(int, "collect_angle", collect_angle, 0);
	// alarm_msg
	READ_PARAM(bool, "alarm_msg", alarm_msg, false);
	//log
	READ_PARAM(bool, "Savelog", Savelog, false);
	READ_PARAM(std::string, "logPath", logPath, std::string("/opt/log"));

	READ_PARAM(int, "error_circle", error_circle, 3);
	READ_PARAM(double, "error_scale", error_scale, 0.9);
	READ_PARAM(int, "direction", direction, -1);

	std::vector<Range> custom_masks;

	uint32_t device_ability = get_device_ability(platform);

	//
	uint32_t init_states = 0;
	if (unit_is_mm)
		init_states |= DF_UNIT_IS_MM;
	if (with_confidence)
		init_states |= DF_WITH_INTENSITY;
	if (hard_resample)
		init_states |= DF_WITH_RESAMPLE;
	if (with_smooth)
		init_states |= DF_SMOOTHED;
	if (with_deshadow)
		init_states |= DF_DESHADOWED;
	if (alarm_msg)
		init_states |= EF_ENABLE_ALARM_MSG;

	HParser parsers[MAX_LIDARS];
	PubHub *hubs[MAX_LIDARS] = {NULL};
	for (int i = 0; i < lidar_count; i++)
	{
		parsers[i] = ParserOpen(raw_bytes, device_ability, init_states, init_rpm, resample_res,
								with_chk, dev_id, error_circle, error_scale,direction);
		hubs[i] = new PubHub;
		hubs[i]->nfan = 0;
		pthread_mutex_init(&hubs[i]->mtx, NULL);
	}

	if (g_type == "uart")
	{
		int *rates = new int[rate_list.size() + 1];
		for (int i = 0; i < (int)rate_list.size(); i++)
		{
			rates[i] = rate_list[i];
			printf("[%d] => %d\n", i, rate_list[i]);
		}
		rates[rate_list.size()] = 0;
		g_reader = StartUartReader(g_type.c_str(),port.c_str(), baud_rate, rates, parsers[0], hubs[0],Savelog,logPath.c_str());
	}
	else if (g_type == "udp")
	{
		LidarInfo lidars[MAX_LIDARS];
		for (int i = 0; i < lidar_count; i++)
		{
			lidars[i].parser = parsers[i];
			lidars[i].pub = hubs[i];
			strcpy(lidars[i].lidar_ip, lidar_ips[i].c_str());
			lidars[i].lidar_port = lidar_ports[i];
		}
		g_reader = StartUDPReader(g_type.c_str(),local_port, is_group_listener, group_ip.c_str(), lidar_count, lidars,Savelog,logPath.c_str());
	}
	else if (g_type == "tcp")
	{
		g_reader = StartTCPReader(lidar_ips[0].c_str(), lidar_ports[0], parsers[0], hubs[0]);
	}

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pubs[MAX_LIDARS];
	for (int i = 0; i < lidar_count; i++)
	{
		if (output_scan)
		{
			laser_pubs[i] = node->create_publisher<sensor_msgs::msg::LaserScan>(laser_topics[i], rclcpp::SensorDataQoS());
		}
	}
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service = node->create_service<std_srvs::srv::Empty>("start", &start_motor);
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service2 = node->create_service<std_srvs::srv::Empty>("stop", &stop_motor);
	rclcpp::WallRate loop_rate(100);
	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		bool idle = true;
		for (int i = 0; i < lidar_count; i++)
		{
			RawData *fans[MAX_FANS];

			if (!output_360)
			{
				if (GetFan(hubs[i], with_soft_resample, resample_res, fans))
				{
					if (output_scan)
					{
						PublishLaserScanFan(laser_pubs[i], fans[0], frame_id,
											min_dist, max_dist,
											with_angle_filter, min_angle, max_angle,
											inverted, reversed, zero_shift,
											custom_masks,collect_angle);
						// printf("free %x\n", fans[0]);
					}
					delete fans[0];
					idle = false;
				}
			}
			else
			{
				uint32_t ts_beg[2], ts_end[2];
				int n = GetAllFans(hubs[i], with_soft_resample, resample_res, fans, from_zero,collect_angle, ts_beg, ts_end);
				if (n > 0)
				{
					idle = false;
					//if (output_scan)
					{
						PublishLaserScan(laser_pubs[i], n, fans, frame_id, min_dist, max_dist,
										 with_angle_filter, min_angle, max_angle,
										 inverted, reversed, zero_shift, from_zero,
										 ts_beg, ts_end, custom_masks,collect_angle);
					}

					for (int i = 0; i < n; i++)
						delete fans[i];
				}
				
			}
		}
		if (idle)
		{
			loop_rate.sleep();
		}
	}

	if (g_type == "uart")
	{
		StopUartReader(g_reader);
	}
	else if (g_type == "udp")
	{
		StopUDPReader(g_reader);
	}
	else if (g_type == "tcp")
	{
		StopTCPReader(g_reader);
	}

	rclcpp::shutdown();
	return 0;
}
