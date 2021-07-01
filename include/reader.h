#ifndef _READER_
#define _READER_

#include "parser.h"

typedef void* HReader;
typedef void* HPublish;


void PublishData(HPublish, int, RawData**);


HReader StartUartReader(const char* port, int baudrate, int* rate_list, HParser, HPublish);
void StopUartReader(HReader);

bool SendUartCmd(HReader, int len, char*);


HReader StartUDPReader(const char* lidar_ip, unsigned short lidar_port, 
		const char* group_ip, unsigned short listen_port, 
		HParser hParser, HPublish hPub);
void StopUDPReader(HReader);

bool SendUdpCmd(HReader hr, int len, char* cmd);

HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub);
void StopTCPReader(HReader);

bool SendTcpCmd(HReader hr, int len, char* cmd);

#endif
