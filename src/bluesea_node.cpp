#include <cstdio>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <pthread.h>

#include <sys/time.h>
// #include "timer.h"
#include "../include/reader.h"
#include "../include/algorithmAPI.h"
#include"../include/bluesea_node.h"
#include <cmath>

#define ROS2Verision "2.0"

HReader g_reader = NULL;
std::string g_type = "uart";


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
	else if (g_type == "vpc")
	{
		return SendVpcCmd(g_reader, len, cmd);
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
	if (hub->nfan > 0)
	{
		pthread_mutex_lock(&hub->mtx);
		fans[0] = hub->fans[0];
		for (int i = 1; i < hub->nfan; i++)
			hub->fans[i - 1] = hub->fans[i];
		hub->nfan--;
		got = true;
		pthread_mutex_unlock(&hub->mtx);
	}
	else
		return false;

	if (with_resample) // && resample_res > 0.05)
	{
		int NN = fans[0]->span / (10 * resample_res);

		if (NN < fans[0]->N)
		{
			resample(fans[0], NN);
		}
		else if (NN > fans[0]->N)
		{
			printf("fan %d less than %d\n", fans[0]->N, NN);
		}
	}
	return got;
}

int GetAllFans(HPublish pub, ArgData argdata, uint8_t &counterclockwise)
{
	PubHub *hub = (PubHub *)pub;
	RawData *fans[MAX_FANS];
	// int checkAngle = (from_zero == 0 ? 0 : 180);
	//  解析出来一帧的数据
	pthread_mutex_lock(&hub->mtx);
	int cnt = 0, nfan = 0;
	for (int i = 1; i < hub->nfan; i++)
	{
		if (hub->fans[i]->angle == hub->offsetangle)
		{

			hub->ts_end[0] = hub->fans[i]->ts[0];
			hub->ts_end[1] = hub->fans[i]->ts[1];
			cnt = i;
			break;
		}
	}
	if (cnt > 0)
	{
		// 起始点位在扇区的中间部分，删除扇区时就需要多保存一个扇区
		if (hub->offsetidx > 0)
		{
			nfan = cnt + 1;
			for (int i = 0; i <= cnt; i++)
			{
				fans[i] = hub->fans[i];
			}
			for (int i = cnt; i < hub->nfan; i++)
				hub->fans[i - cnt] = hub->fans[i];

			hub->nfan -= cnt;
		}
		else
		{
			nfan = cnt;
			for (int i = 0; i < cnt; i++)
			{
				fans[i] = hub->fans[i];
			}
			for (int i = cnt; i < hub->nfan; i++)
				hub->fans[i - cnt] = hub->fans[i];
			hub->nfan -= cnt;
		}
	}
	pthread_mutex_unlock(&hub->mtx);
	// 一帧数据的丢包检测以及软采样角分辨率的适配
	if (cnt > 0)
	{
		counterclockwise = fans[0]->counterclockwise;

		bool circle = true;
		int total = fans[0]->span;
		hub->ts_beg[0] = fans[0]->ts[0];
		hub->ts_beg[1] = fans[0]->ts[1];

		for (int i = 0; i < nfan - 1; i++)
		{
			if ((fans[i]->angle + fans[i]->span) % 3600 != fans[i + 1]->angle)
			{
				circle = false;
			}
			total += fans[i + 1]->span;
		}
		//printf("%d  %d %d \n", circle,total,hub->offsetidx);
		if (!circle || (total != 3600 && hub->offsetidx == 0) || ((total != 3600 + fans[0]->span) && hub->offsetidx > 0))
		{
			// clean imcomplent datas
			for (int i = 0; i < cnt; i++)
				delete fans[i];
			cnt = 0;
		}
	}
	if (cnt > 0)
	{
		if (argdata.soft_resample && argdata.resample > 0)
		{
			for (int i = 0; i < cnt; i++)
			{
				int NN = fans[i]->span / (10 * argdata.resample);
				if (NN < fans[i]->N)
				{
					// printf("%s %d  %d,NN:%d,fans[i]->N:%d  %lf\n",__FUNCTION__,__LINE__,cnt,NN,fans[i]->span,resample_res);
					resample(fans[i], NN);
				}
				else if (NN > fans[i]->N)
				{
					printf("fan [%d] %d less than %d\n", i, fans[i]->N, NN);
				}
			}
		}
		// 将扇区数据转为点 列表

		for (int j = hub->offsetidx; j < fans[0]->N; j++)
		{
			hub->consume.push_back(fans[0]->points[j]);
		}
		for (int i = 1; i < cnt; i++)
		{
			for (int j = 0; j < fans[i]->N; j++)
			{
				hub->consume.push_back(fans[i]->points[j]);
			}
		}
		for (int j = 0; j < hub->offsetidx; j++)
		{
			hub->consume.push_back(fans[cnt]->points[j]);
		}
		for (int i = 0; i < cnt; i++)
			delete fans[i];
		// ROS_INFO("%s %d %f %f\n", __FUNCTION__, __LINE__, hub->consume[0].degree,hub->consume[hub->consume.size()-1].degree);

		if (cnt > 0)
		{
			bool res = checkZeroDistance(hub->consume, argdata.custom.error_scale);
			if (res)
				hub->error_num = 0;
			else
			{
				hub->error_num++;
				if (hub->error_num >= argdata.custom.error_circle)
				{
					printf("There are many points with a distance of 0 in the current lidar operation");
					hub->error_num = 0;
				}
			}
		}

		int N = hub->consume.size();
		double angle_increment = 0;

		if (argdata.from_zero)
		{
			if (argdata.inverted)
				angle_increment = M_PI * 2 / N;
			else
				angle_increment = -M_PI * 2 / N;
		}
		else
		{
			if (argdata.inverted)
				angle_increment = M_PI * 2 / N;
			else
				angle_increment = -M_PI * 2 / N;
		}

		if (counterclockwise)
			angle_increment = -angle_increment;
		Fitter *fitter = &argdata.fitter;
		//ROS_DEBUG("%d %d %f %f %f %d %lf\n",fitter->isopen,fitter->type, fitter->max_range, fitter->min_range, fitter->max_range_difference, fitter->filter_window,angle_increment);
		if (fitter->isopen)
			filter(hub->consume, fitter->type, fitter->max_range*1000, fitter->min_range*1000, fitter->max_range_difference*1000, fitter->filter_window, angle_increment);
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

int GetCount(std::vector<DataPoint> data, double min_deg, double max_deg, double &min_pos, double &max_pos)
{
	int N = 0;

	for (unsigned int i = 0; i < data.size(); i++)
	{
		double deg = ROSAng(data[i].degree);

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

	// printf("angle filter [%f, %f] %d to %d, [%f, %f]\n", min_deg, max_deg, cnt, N, min_pos, max_pos);
	return N;
}

void PublishLaserScanFan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub, RawData *fan, std::string &frame_id, double min_dist, double max_dist, uint8_t inverted, uint8_t reversed)
{
	sensor_msgs::msg::LaserScan msg;
	msg.header.stamp.sec = fan->ts[0];
	msg.header.stamp.nanosec = fan->ts[1];
	msg.header.frame_id = frame_id;

	int N = fan->N;
	double scan_time = 1 / 100.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / fan->N;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;
	double min_pos, max_pos;
	msg.intensities.resize(N); // fan->N);
	msg.ranges.resize(N);	   // fan->N);
	if (inverted)
	{
		min_pos = ROSAng(-fan->angle / 10) * 10 * M_PI / 1800;
		max_pos = -min_pos; // + fan->span * M_PI / 1800;
	}
	else
	{
		min_pos = ROSAng(fan->angle / 10) * 10 * M_PI / 1800;
		max_pos = min_pos; //-fan->span  * M_PI / 1800;
	}
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
	N = 0;
	if (reversed)
	{
		for (int i = fan->N - 1; i >= 0; i--)
		{
			//double deg = ROSAng(fan->points[i].degree);

			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist)
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
			//double deg = ROSAng(fan->points[i].degree);
			// customize angle filter
			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist)
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
void PublishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  cloud_pub, int nfan, RawData** fans, std::string& frame_id, 
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

void PublishLaserScan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub,HPublish pub, ArgData argdata, uint8_t counterclockwise)
{
	PubHub *hub = (PubHub *)pub;
	sensor_msgs::msg::LaserScan msg;
	int N = hub->consume.size();
	// make  min_ang max_ang  convert to mask
	msg.header.stamp.sec = hub->ts_beg[0];
	msg.header.stamp.nanosec  = hub->ts_beg[1];

	double ti = double(hub->ts_beg[0]) + double(hub->ts_beg[1]) / 1000000000.0;
	double tx = double(hub->ts_end[0]) + double(hub->ts_end[1]) / 1000000000.0;

	msg.scan_time = tx - ti; // nfan/(nfan-1);
	msg.time_increment = msg.scan_time / N;

	msg.header.frame_id = argdata.frame_id;

	double min_deg = argdata.min_angle * 180 / M_PI;
	double max_deg = argdata.max_angle * 180 / M_PI;

	msg.range_min = argdata.min_dist;
	msg.range_max = argdata.max_dist; // 8.0;
	if (argdata.with_angle_filter)
	{
		double min_pos, max_pos;
		N = GetCount(hub->consume, min_deg, max_deg, min_pos, max_pos);
		if (argdata.inverted)
		{
			msg.angle_min = min_pos * M_PI / 180;
			msg.angle_max = max_pos * M_PI / 180;
			// printf("data1:deg:%lf   %lf  angle:%lf   %lf   %d\n",min_deg,max_deg,msg.angle_min,msg.angle_max,N);
		}
		else
		{
			msg.angle_min = max_pos * M_PI / 180;
			msg.angle_max = min_pos * M_PI / 180;
		}
		msg.angle_increment = (msg.angle_max - msg.angle_min) / (N - 1);
	}
	else if (argdata.from_zero)
	{
		if (argdata.inverted)
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
		if (argdata.inverted)
		{
			msg.angle_min = -M_PI;
			msg.angle_max = M_PI - (2 * M_PI) / N;
			msg.angle_increment = M_PI * 2 / N;
		}
		else
		{
			msg.angle_min = M_PI;
			msg.angle_max = -M_PI + (2 * M_PI) / N;
			msg.angle_increment = -M_PI * 2 / N;
		}
	}

	if (counterclockwise != 0)
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
	int idx = 0;

	if (argdata.reversed)
	{
		for (int i = N - 1; i >= 0; i--)
		{
			double deg = ROSAng(hub->consume[i].degree);
			double d = hub->consume[i].distance / 1000.0;
			bool custom = false;
			for (unsigned int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = hub->consume[i].confidence;
			idx++;
		}
	}
	else
	{

		// ROS_INFO("%s %d %f %f\n", __FUNCTION__, __LINE__, hub->consume[0].degree,hub->consume[hub->consume.size()-1].degree);
		for (int i = 0; i <= N - 1; i++)
		{
			// printf(" %d %d  %d  %lf \n", datapoint[i].distance,datapoint[i].confidence,N,datapoint[i].degree);
			double deg = ROSAng(hub->consume[i].degree);
			double d = hub->consume[i].distance / 1000.0;
			bool custom = false;
			for (unsigned int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = hub->consume[i].confidence;

			idx++;
		}
	}
	hub->consume.clear();
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
	VAR=INIT;								  \
	node->declare_parameter<TYPE>(NAME, VAR); \
	node->get_parameter(NAME, VAR);

bool get_range_param(std::shared_ptr<rclcpp::Node> node, const char *name, Range &range)
{
	std::string mask;
	READ_PARAM(std::string, name, mask, "");
	if (mask != "")
	{
		int index = mask.find(",");
		float min = atof(mask.substr(0, index).c_str());
		float max = atof(mask.substr(index + 1).c_str());
		if (min < max)
		{
			range.min = min * 180 / M_PI;
			range.max = max * 180 / M_PI;
			return true;
		}
	}
	return false;
}

void getCMDList(CommandList &cmdlist, ArgData argdata)
{

	if (argdata.uuid >= 0)
		sprintf(cmdlist.uuid, "LUUIDH");
	if (argdata.model >= 0)
		sprintf(cmdlist.model, "LTYPEH");
	if (argdata.rpm >= 0)
		sprintf(cmdlist.rpm, "LSRPM:%dH", argdata.rpm);
	if (argdata.resample >= 0)
	{

		if (argdata.resample > 0 && argdata.resample < 1)
			sprintf(cmdlist.res, "LSRES:%dH", (int)(argdata.resample * 1000));
		else
			sprintf(cmdlist.res, "LSRES:%dH", (int)argdata.resample);
	}
	if (argdata.with_smooth >= 0)
	{
		if (argdata.type == "uart")
			sprintf(cmdlist.smooth, "LSSS%dH", argdata.with_smooth);
		else
			sprintf(cmdlist.smooth, "LSSMT:%dH", argdata.with_smooth);
	}
	if (argdata.with_deshadow >= 0)
	{
		if (argdata.type == "uart")
			sprintf(cmdlist.fitter, "LFFF%dH", argdata.with_deshadow);
		else
			sprintf(cmdlist.fitter, "LSDSW:%dH", argdata.with_deshadow);
	}
	if (argdata.alarm_msg >= 0)
		sprintf(cmdlist.alarm, "LSPST:%dH", argdata.alarm_msg ? 3 : 1);
	if (argdata.direction >= 0)
		sprintf(cmdlist.direction, "LSCCW:%dH", argdata.direction);
	if (argdata.unit_is_mm >= 0)
		sprintf(cmdlist.unit_mm, "%s", argdata.unit_is_mm ? "LMDMMH" : "LMDCMH");
	if (argdata.with_confidence >= 0)
		sprintf(cmdlist.confidence, "%s", argdata.with_confidence ? "LOCONH" : "LNCONH");

	if (argdata.ats >= 0)
	{
		if (argdata.type == "vpc" || argdata.type == "uart")
			sprintf(cmdlist.ats, "LSATS:002H");
		else if (argdata.type == "udp")
			sprintf(cmdlist.ats, "LSATS:001H");
	}
}

bool autoGetFirstAngle2(HPublish pub, bool from_zero)
{
	PubHub *hub = (PubHub *)pub;
	int checkAngle = (from_zero == true ? 0 : 180);
	for (int i = 0; i < hub->nfan; i++)
	{
		int idx = getFirstidx(*(hub->fans[i]), checkAngle);
		if (idx >= 0)
		{
			//ROS_INFO("%s %d %d %d\n", __FUNCTION__, __LINE__, hub->fans[i]->angle, checkAngle);
			hub->offsetangle = hub->fans[i]->angle;
			hub->offsetidx = idx;
			return true;
		}
	}
	return false;
}

bool ProfileInit(std::shared_ptr<rclcpp::Node>node, ArgData &argdata)
{
	READ_PARAM(int, "number", argdata.num, 1);
	READ_PARAM(std::string, "type", argdata.type, "uart");
	READ_PARAM(std::string, "frame_id", argdata.frame_id, "LH_laser");
	READ_PARAM(int, "dev_id", argdata.dev_id, ANYONE);

	printf("argdata.num %d\n",argdata.num);
	// dual lidar arg
	for (int i = 0; i <argdata.num; i++)
	{
		ConnectArg arg;
		char s[32], t[32];
		if (i == 0)
		{
			if (argdata.type == "udp")
			{
				READ_PARAM(std::string,"lidar_ip", arg.arg1, std::string("192.168.158.98"));
				READ_PARAM(int,"lidar_port", arg.arg2, 6543);
			}
			else if (argdata.type == "vpc")
			{
				READ_PARAM(std::string,"port", arg.arg1, std::string("/dev/ttyACM0"));
				READ_PARAM(int,"baud_rate", arg.arg2, 115200);
			}
			else if (argdata.type == "uart")
			{
				READ_PARAM(std::string,"port", arg.arg1, std::string("/dev/ttyUSB0"));
				READ_PARAM(int,"baud_rate", arg.arg2, 500000);
			}
			else
			{
				ROS_ERROR("Profiles type=%s is not exist!", argdata.type.c_str());
				return false;
			}

			READ_PARAM(std::string,"topic", arg.laser_topics, std::string("scan"));
			READ_PARAM(std::string,"cloud_topic", arg.cloud_topics, std::string("cloud"));
		}
		else
		{
			if (argdata.type == "udp")
			{
				//printf("222\n");
				sprintf(s, "lidar%d_ip", i);
				READ_PARAM(std::string,s, arg.arg1, std::string(""));
				sprintf(s, "lidar%d_port", i);
				READ_PARAM(int,s, arg.arg2, 0);
				if (arg.arg1.empty() || arg.arg2 == 0)
				{
					ROS_ERROR("Profiles lidar num %d is not exist!", i);
					break;
				}
				sprintf(s, "topic%d", i);
				sprintf(t, "scan%d", i);
				READ_PARAM(std::string,s, arg.laser_topics, std::string(t));
				sprintf(s, "cloud_topic%d", i);
				sprintf(t, "cloud%d", i);
				READ_PARAM(std::string,s, arg.cloud_topics, std::string(t));
			}
		}
		argdata.connectargs.push_back(arg);
	}
	READ_PARAM(int,"local_port", argdata.localport, 6668);
	// custom
	READ_PARAM(bool,"group_listener", argdata.custom.is_group_listener, false);
	READ_PARAM(std::string,"group_ip", argdata.custom.group_ip, std::string("224.1.1.91"));
	READ_PARAM(int,"raw_bytes", argdata.raw_bytes, 3); // packet mode : 2bytes or 3bytes

	READ_PARAM(bool,"inverted", argdata.inverted, false);
	READ_PARAM(bool,"reversed", argdata.reversed, false);
	// data output
	READ_PARAM(bool,"output_scan", argdata.output_scan, true);	// true: enable output angle+distance mode, 0: disable
	READ_PARAM(bool,"output_cloud", argdata.output_cloud, false); // false: enable output xyz format, 0 : disable
	READ_PARAM(bool,"output_360", argdata.output_360, true);		// true: packet data of 360 degree (multiple RawData), publish once
																// false: publish every RawData (36 degree)
	//  angle filter
	READ_PARAM(bool,"with_angle_filter", argdata.with_angle_filter, false); // true: enable angle filter, false: disable
	READ_PARAM(double,"min_angle", argdata.min_angle, -M_PI);				  // angle filter's low threshold, default value: -pi
	READ_PARAM(double,"max_angle", argdata.max_angle, M_PI);				  // angle filters' up threashold, default value: pi

	// range limitation
	READ_PARAM(double,"min_dist", argdata.min_dist, 0.1);  // min detection range, default value: 0M
	READ_PARAM(double,"max_dist", argdata.max_dist, 50.0); // max detection range, default value: 9999M
	// customize angle filter
	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(node, name, range))
			break;
		argdata.masks.push_back(range);
	}
	READ_PARAM(bool,"from_zero", argdata.from_zero, false);
	READ_PARAM(int, "error_circle", argdata.custom.error_circle, 3);
	READ_PARAM(double,"error_scale", argdata.custom.error_scale, 0.9);

	/*******************************FITTER arg start******************************/
	READ_PARAM(bool,"filter_open", argdata.fitter.isopen, false);
	READ_PARAM(int,"filter_type", argdata.fitter.type, 1);
	READ_PARAM(double,"max_range", argdata.fitter.max_range, 20.0);
	READ_PARAM(double,"min_range", argdata.fitter.min_range, 0.5);
	READ_PARAM(double,"max_range_difference", argdata.fitter.max_range_difference, 0.1);
	READ_PARAM(int,"filter_window", argdata.fitter.filter_window, 1);
	/*******************************FITTER arg end******************************/

	/*****************************GET arg start************************************/
	READ_PARAM(int,"uuid", argdata.uuid, -1);
	READ_PARAM(int,"model", argdata.model, -1);
	/*****************************GET arg end************************************/
	/*****************************SET arg start************************************/
	READ_PARAM(int,"rpm", argdata.rpm, -1); // set motor RPM
	// angle composate
	READ_PARAM(bool,"hard_resample", argdata.hard_resample, false); // resample angle resolution
	READ_PARAM(bool,"soft_resample", argdata.soft_resample, false); // resample angle resolution
	READ_PARAM(double,"resample_res", argdata.resample, -1.0);		  // resample angle resolution

	if (!argdata.hard_resample)
		argdata.resample = -1;

	READ_PARAM(int,"with_smooth", argdata.with_smooth, -1);	   // lidar data smooth filter
	READ_PARAM(int,"with_deshadow", argdata.with_deshadow, -1); // data shadow filter
	READ_PARAM(int,"alarm_msg", argdata.alarm_msg, -1);		   // let lidar upload alarm message
	READ_PARAM(int,"direction", argdata.direction, -1);
	READ_PARAM(int,"unit_is_mm", argdata.unit_is_mm, -1); // 0 : distance is CM, 1: MM
	READ_PARAM(int,"with_confidence", argdata.with_confidence, -1);
	READ_PARAM(int,"ats", argdata.ats, -1);
	/*****************************SET arg end************************************/
	return true;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node>node = rclcpp::Node::make_shared("bluesea_node");

	rclcpp::QoS qos(0);
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
	// std::shared_ptr<rclcpp::Node>ptr = rclcpp::Node::make_shared("bluesea_node");
	signal(SIGINT, closeSignal);
	ArgData argdata;
	ProfileInit(node, argdata);
	
	CommandList cmdlist;
	memset(&cmdlist, 0, sizeof(CommandList));
	getCMDList(cmdlist, argdata);
	printf("ROS2 Verision:%s\n", ROS2Verision);


	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pubs[MAX_LIDARS];
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service = node->create_service<std_srvs::srv::Empty>("start", &start_motor);
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service2 = node->create_service<std_srvs::srv::Empty>("stop", &stop_motor);
	rclcpp::WallRate loop_rate(100);

	HParser parsers[MAX_LIDARS];
	PubHub *hubs[MAX_LIDARS] = {NULL};
	for (int i = 0; i < argdata.num; i++)
	{
		parsers[i] = ParserOpen(argdata.raw_bytes, true, argdata.dev_id, argdata.custom.error_circle, argdata.custom.error_scale, argdata.from_zero, cmdlist, (char *)argdata.connectargs[i].arg1.c_str(), argdata.connectargs[i].arg2);
		hubs[i] = new PubHub;
		hubs[i]->nfan = 0;
		hubs[i]->offsetangle = -1;
		pthread_mutex_init(&hubs[i]->mtx, NULL);
		if (argdata.output_scan)
			laser_pubs[i] = node->create_publisher<sensor_msgs::msg::LaserScan>(argdata.connectargs[i].laser_topics, qos);

	}

	if (argdata.type == "uart" || argdata.type == "vpc")
	{
		g_reader = StartUartReader(argdata.type.c_str(), argdata.connectargs[0].arg1.c_str(), argdata.connectargs[0].arg2, parsers[0], hubs[0]);
	}
	else if (argdata.type == "udp")
	{
		LidarInfo lidars[MAX_LIDARS];
		for (int i = 0; i < argdata.num; i++)
		{
			lidars[i].parser = parsers[i];
			lidars[i].pub = hubs[i];
			strcpy(lidars[i].lidar_ip, argdata.connectargs[i].arg1.c_str());
			lidars[i].lidar_port = argdata.connectargs[i].arg2;
		}
		g_reader = StartUDPReader(argdata.type.c_str(), argdata.localport, argdata.custom.is_group_listener, argdata.custom.group_ip.c_str(), argdata.num, lidars);
	}
	else if (argdata.type == "tcp")
	{
		g_reader = StartTCPReader(argdata.connectargs[0].arg1.c_str(), argdata.connectargs[0].arg2, parsers[0], hubs[0]);
	}


	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		bool idle = true;
		for (int i = 0; i < argdata.num; i++)
		{
			if (hubs[i]->nfan == 0)
				continue;
			bool ret = false;
			if (hubs[i]->offsetangle == -1)
			{
				ret = autoGetFirstAngle2(hubs[i], argdata.from_zero);
				if (ret > 0)
				{
					ROS_INFO("lidar start work,offset angle %d offset idx %d\n", hubs[i]->offsetangle/10, hubs[i]->offsetidx);
				}
				continue;
			}
			if (!argdata.output_360)
			{
				RawData *fans[MAX_FANS] = {NULL};
				if (GetFan(hubs[i], argdata.soft_resample, argdata.resample, fans))
				{
					if (argdata.output_scan)
					{
						PublishLaserScanFan(laser_pubs[i], fans[0], argdata.frame_id,
											argdata.min_dist, argdata.max_dist, argdata.inverted, argdata.reversed);
					}
					delete fans[0];
					idle = false;
				}
			}
			else
			{
				// 按照点数判定一帧的数据，然后   零数判定， 离异点过滤，最大最小值过滤(空点去除)， 指定角度屏蔽(mask置零)
				uint8_t counterclockwise = false;
				int n = GetAllFans(hubs[i], argdata, counterclockwise);
				if (n > 0)
				{
					idle = false;
					if (argdata.output_scan)
					{
						PublishLaserScan(laser_pubs[i], hubs[i], argdata, counterclockwise);
					}

					if (argdata.output_cloud)
					{
						//PublishCloud(cloud_pubs[i], hubs[i], argdata);
					}
				}
			}
		}
		if (idle)
		{
			loop_rate.sleep();
		}
	}
	return 0;
}
