#ifndef _ALGORITHMAPI_
#define _ALGORITHMAPI_ 
#include"common_data.h"


//离异点过滤
bool checkWindowValid2(std::vector<DataPoint>scan, size_t idx, size_t window, double max_distance,double angle_increment);
bool filter(std::vector<DataPoint>& output_scan,double max_range,double min_range,double max_range_difference,unsigned int filter_window,double angle_increment);
//判定一圈的点数中距离为0的点位的比例是否符合要求
bool checkZeroDistance(std::vector<DataPoint>comsume,float error_scale);
//CRC算法
unsigned int stm32crc(unsigned int *ptr, unsigned int len);
//软采样设置
void resample(RawData *dat, int NN);
#endif
