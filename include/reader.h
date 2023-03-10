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

HReader StartUartReader(const char* type ,const char* port, int baudrate, int* rate_list, HParser, HPublish,bool isSaveLog,const char* logPath);
bool SendUartCmd(HReader, int len, char*);
bool SendVpcCmd(HReader hr, int len, char* cmd);


HReader StartUDPReader(const char* type,unsigned short listen_port, bool is_group_listener, const char* group_ip,
	int lidar_count, const LidarInfo* lidars,bool isSaveLog,const char* logPath);


bool SendUdpCmd(HReader hr, int id, int len, char* cmd);
bool SendUdpCmd2(HReader hr, char* ip, int len, char* cmd);
HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub);

bool SendTcpCmd(HReader hr, int len, char* cmd);

void StopUartReader(HReader hr);
void StopUDPReader(HReader hr);
void StopTCPReader(HReader hr);
//void saveLog(bool isSaveLog,const char*logPath,int type,const unsigned char*buf,unsigned int len);
bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, bool isSaveLog, const char *logPath,void* result);
bool udp_talk_C_PACK(void *hnd,int n, const char *cmd,int nhdr, const char *hdr_str,int nfetch, char *fetch);
bool udp_talk_S_PACK(void *hnd, int n, const char *cmd,void *result);
#endif
