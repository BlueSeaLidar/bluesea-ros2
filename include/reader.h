#ifndef _READER_
#define _READER_

#include "parser.h"




#define MAX_LIDARS 8


struct LidarInfo {
	HParser parser;
	HPublish pub;
	char lidar_ip[32];
	int lidar_port;
};
void PublishData(HPublish, int, RawData**);

HReader StartUartReader(const char* port, int baudrate, int* rate_list, HParser, HPublish);
bool SendUartCmd(HReader, int len, char*);


HReader StartUDPReader(const char* lidar_ip, unsigned short lidar_port, unsigned short listen_port, 
		bool is_group_listener, const char* group_ip, 
		HParser hParser, HPublish hPub);

HReader StartUDPReader( unsigned short listen_port, bool is_group_listener, const char* group_ip,
	int lidar_count, const LidarInfo* lidars);


bool SendUdpCmd(HReader hr, int id, int len, char* cmd);

HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub);

bool SendTcpCmd(HReader hr, int len, char* cmd);

void StopUartReader(HReader hr);
void StopUDPReader(HReader hr);
void StopTCPReader(HReader hr);


#endif
