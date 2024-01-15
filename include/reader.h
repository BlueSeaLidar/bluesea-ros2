#ifndef _READER_
#define _READER_

#include "parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#define MAX_LIDARS 8
#define GS_PACK 0x4753
#define S_PACK 0x0053
#define C_PACK 0x0043

#define ROS_ERROR printf
#define ROS_INFO printf
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

HReader StartUartReader(const char* type ,const char* port, int baudrate, HParser, HPublish);
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


//UART
int open_serial_port(const char *port, int baudrate);
bool uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch,int waittime=10,int cachelength=4096);



//VPC
bool vpc_talk(int hcom, int mode, short sn, int len, const char* cmd, int nfetch, void* result);


//udp
bool setup_lidar_udp(HParser hP, int handle);
void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,int n, const char* cmd,int nhdr, const char* hdr_str,int nfetch, char* fetch);
bool udp_talk_GS_PACK(int fd_udp, const char *ip, int port, int n, const char *cmd, void *result);

//common

#endif
