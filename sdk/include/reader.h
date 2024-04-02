#ifndef _READER_
#define _READER_

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
#include "parser.h"

HReader StartUartReader(const char* type ,const char* port, int baudrate, Parser*hParser, PubHub*);
bool SendUartCmd(HReader, int len, char*);
bool SendVpcCmd(HReader hr, int len, char* cmd);

HReader StartUDPReader(const char* type,unsigned short listen_port, bool is_group_listener, const char* group_ip,
	int lidar_count, const LidarInfo* lidars);


bool SendUdpCmd(HReader hr, int id, int len, char* cmd);
bool SendUdpCmd2(HReader hr, char* ip, int len, char* cmd);
HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, Parser* hParser, PubHub* hPub);

bool SendTcpCmd(HReader hr, int len, char* cmd);

void StopUartReader(HReader hr);
void StopUDPReader(HReader hr);
void StopTCPReader(HReader hr);


//UART
int open_serial_port(const char *port, int baudrate);
bool uart_talk(int fd, int n, const char* cmd, int nhdr, const char* hdr_str, int nfetch, char* fetch,int waittime=10,int cachelength=4096);
int GetComBaud(std::string uartname);


//VPC
bool vpc_talk(int hcom, int mode, short sn, int len, const char* cmd, int nfetch, void* result);


//udp
bool setup_lidar_udp(int handle,Parser* hP, EEpromV101 &param);
void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf);
bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result);
bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,int n, const char* cmd,int nhdr, const char* hdr_str,int nfetch, char* fetch);
bool udp_talk_GS_PACK(int fd_udp, const char *ip, int port, int n, const char *cmd, void *result);

//common
void PublishData(PubHub* pub, int n, RawData **fans);
#endif
