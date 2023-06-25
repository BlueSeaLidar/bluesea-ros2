#ifndef _READER_
#define _READER_

#include "parser.h"
#define MAX_LIDARS 8

struct UartInfo
{
	char type[8]; // uart or vpc
	int fd_uart;
	char port[128];
	int baudrate;
	int *rate_list;

	HParser hParser;
	HPublish hPublish;
	pthread_t thr;
};

struct UDPInfo
{
	char type[8]; //"udp"
	int nnode;
	LidarNode lidars[MAX_LIDARS];

	int fd_udp;
	int listen_port;
	bool is_group_listener;
	pthread_t thr;

};

struct ScriptParam
{
	UDPInfo *info;
	int id;
};

struct KeepAlive
{
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};

struct LidarInfo {
	HParser parser;
	HPublish pub;
	char lidar_ip[32];
	int lidar_port;
};
void PublishData(HPublish, int, RawData**);

HReader StartUartReader(const char* type ,const char* port, int baudrate, int* rate_list, HParser, HPublish);
bool SendUartCmd(HReader, int len, char*);
bool SendVpcCmd(HReader hr, int len, char* cmd);


HReader StartUDPReader(const char* type,unsigned short listen_port, bool is_group_listener, const char* group_ip,
	int lidar_count, const LidarInfo* lidars);


bool SendUdpCmd(HReader hr, int id, int len, char* cmd);
bool SendUdpCmd2(HReader hr, char* ip, int len, char* cmd);
HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub);

bool SendTcpCmd(HReader hr, int len, char* cmd);

void StopUartReader(HReader hr);
void StopUDPReader(HReader hr);
void StopTCPReader(HReader hr);
void saveLog(const char*logPath,int type,const unsigned char*buf,unsigned int len);
bool udp_talk_GS_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result,const char *logPath);
bool udp_talk_C_PACK(int fd_udp, const char* ip, int port,int n, const char *cmd,int nhdr, const char *hdr_str,int nfetch, char *fetch,const char *logPath);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char *cmd,void *result,const char *logPath);


bool uart_talk(int fd,int n, const char *cmd,int nhdr, const char *hdr_str,int nfetch, char *fetch,const char*logpath);
bool vpc_talk(int hCom, int mode,int len, const char *cmd, int nfetch, void *result, const char *logpath);


#endif
