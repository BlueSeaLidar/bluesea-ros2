#include <stdio.h>
#include <stdlib.h>
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

#include "reader.h"

struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};


struct UDPInfo 
{
	int nnode;
	LidarNode lidars[MAX_LIDARS];

	int fd_udp;
	int listen_port;
	bool is_group_listener;
	pthread_t thr;
};

struct ScriptParam
{
	UDPInfo* info;
	int id;
};

struct KeepAlive {
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};
	
// CRC32
unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}

// 
bool send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
			int len, const void* snd_buf, 
			bool bpr)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len/4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

	if (bpr) {
		char s[3096];
		for (int i = 0; i < len2; i++) 
			sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n", 
				dev_ip, dev_port, cmd, sn, len, s);
	}
	return true;
}

bool send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
		int len, const void* snd_buf)
{
	//printf("send cmd %x len %d : %s\n", cmd, len, (char*)snd_buf);
	return send_cmd_udp_f(fd_udp, dev_ip, dev_port, cmd, sn, len, snd_buf, true);
}

bool udp_config(void* hnd, int n, const char* cmd)
{
	ScriptParam* param = (ScriptParam*)hnd;
	UDPInfo* info = param->info;
	//int fd_udp = info->fd_udp;
	int fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	printf("send config : \'%s\' \n", cmd);

	unsigned short sn = rand();

	int rt = send_cmd_udp(fd_udp, info->lidars[param->id].ip, info->lidars[param->id].port, 
		0x0053, sn, n, cmd);

	int ntry = 0;

	time_t tst = time(NULL);
	while (time(NULL) < tst + 3)
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_udp, &fds); 
	
		struct timeval to = { 2, 0 }; 
		int ret = select(fd_udp+1, &fds, NULL, NULL, &to); 

		if (ret <= 0) {
			close(fd_udp); return false;
		}
		
		// read UDP data
		if (FD_ISSET(fd_udp, &fds)) 
		{ 
			ntry++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
	
			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0) 
			{	
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn) continue;

				const int CODE_OK = 0x4b4f;
				close(fd_udp);
				return *(int*)(buf+sizeof(CmdHeader)) == CODE_OK;
			}
		}
	}

	printf("read %d packets, not response\n", ntry);
	close(fd_udp);
	return false;
}

bool udp_talk(void* hnd,
	       	int n, const char* cmd, 
			   int nhdr, const char* hdr_str, 
			   int nfetch, char* fetch)
{
	// configuration 
	if (strstr(cmd, "LSPST")) {
	       	return udp_config(hnd, n, cmd);
	}

	ScriptParam* param = (ScriptParam*)hnd;
	UDPInfo* info = param->info;
	//int fd_udp = info->fd_udp;
	int fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	printf("send command : \'%s\' \n", cmd);

	unsigned short sn = rand();

	int rt = send_cmd_udp(fd_udp, info->lidars[param->id].ip, info->lidars[param->id].port,
		 0x0043, sn, n, cmd);

	int ntry = 0;
	time_t tst = time(NULL);
	while (time(NULL) < tst + 3)
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_udp, &fds); 
	
		struct timeval to = { 1, 0 }; 
		int ret = select(fd_udp+1, &fds, NULL, NULL, &to); 

		if (ret <= 0) {
			close(fd_udp); return false;
		}
		
		// read UDP data
		if (FD_ISSET(fd_udp, &fds)) 
		{ 
			ntry++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);
	
			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0) 
			{	
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn) continue;
					
				for (int i=0; i<nr-nhdr-1; i++) 
				{
					if (memcmp(buf+i, hdr_str, nhdr) == 0) 
					{ 
						//memcpy(fetch, buf+i+nhdr, nfetch);
						//fetch[nfetch] = 0;
						memset(fetch, 0, nfetch);
						for (int j=0; j<nfetch && i+nhdr+j<nr; j++)
							fetch[j] = buf[i+nhdr+j];			
							close(fd_udp); return true;
				       	}
			       	}

				memcpy(fetch, "ok", 2);
				fetch[2] = 0;
			
				close(fd_udp); return true;
			}
		}
	}

	printf("read %d packets, not response\n", ntry);
			
	close(fd_udp); return false;
}

void* UdpThreadProc(void* p)
{
	UDPInfo* info = (UDPInfo*)p;

	int fd_udp = info->fd_udp;
	
	// send requirement to lidar
	if (!info->is_group_listener) 
	{
		for (int i=0; i<info->nnode; i++) {
			char cmd[12] = "LUUIDH";
			int rt = send_cmd_udp(fd_udp,
				info->lidars[i].ip, info->lidars[i].port, 
				0x0043, rand(), 6, cmd);
		}
	}

	char buf[1024];

	timeval tv;
	gettimeofday(&tv, NULL);

	time_t tto = tv.tv_sec + 1;

	uint32_t delay = 0;

	if (!info->is_group_listener) 
	{
		for (int i=0; i<info->nnode; i++)
		{
			ScriptParam param;
			param.info = info;
			param.id = i;
			ParserScript(info->lidars[i].hParser, udp_talk, &param);
		}
	}

	while (1) 
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_udp, &fds); 
	
		struct timeval to = { 1, 5 }; 
		int ret = select(fd_udp+1, &fds, NULL, NULL, &to); 

		if (ret == 0) 
		{
			if (!info->is_group_listener)
			{
				for (int i=0; i<info->nnode; i++)
				{
					char cmd[12] = "LGCPSH";
					int rt = send_cmd_udp(info->fd_udp, 
						info->lidars[i].ip, info->lidars[i].port, 
						0x0043, rand(), 6, cmd);
				}
			}
		}
		
		if (ret < 0) {
			printf("select error\n");
			continue;
		}

		gettimeofday(&tv, NULL);

		// read UDP data
		if (FD_ISSET(fd_udp, &fds)) 
		{ 
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0) 
			{
				int id = -1;
				for (int i=0; i<info->nnode; i++) 
				{
					if (addr.sin_addr.s_addr == info->lidars[i].s_addr)
					{
						id = i;
						break;
					}
				}
				//if (id == -1 && info->nnode == 1) id = 0;

				if (id == -1) {
					printf("packet from unknown address %s\n", inet_ntoa(addr.sin_addr));
				}
				else if (buf[0] == 0x4c && buf[1] == 0x48 && buf[2] == ~0x41 && buf[3] == ~0x4b) 
				{
					if (nr == sizeof(KeepAlive)+12) 
					{
						uint32_t clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec/1000;
						KeepAlive* ka = (KeepAlive*)(buf+8);
						if (clock >= ka->world_clock) 
							delay = clock - ka->world_clock;
						else	
							delay = clock + 36000000 - ka->world_clock;
					}
					//printf("lidar[%d] delay %d\n", id, delay);
				} else {
					//printf("udp %02x%02x\n", buf[0], buf[1]);
					RawData* fans[MAX_FANS];
					//printf("Currnet Lidar IP: %s,Port:%d\n",(char *)inet_ntoa(addr.sin_addr), htons(addr.sin_port));
					//int nfan = ParserRun(info->lidars[id].hParser, nr, (uint8_t*)buf, &(fans[0]));
					int nfan = ParserRun(info->lidars[id], nr, (uint8_t*)buf, &(fans[0]));
					//for (int i=0; i<nfan; i++)
					//	 printf("fan %x %d + %d\n", fans[i], fans[i]->angle, fans[i]->span);
					if (nfan > 0) {
						PublishData(info->lidars[id].hPublish, nfan, fans);
					}
				}
			}
		}

		if (tv.tv_sec > tto && !info->is_group_listener) 
		{
			for (int i=0; i<info->nnode; i++)
			{
				KeepAlive alive;
				gettimeofday(&tv, NULL);
				alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec/1000;
				alive.delay = delay;

				// acknowlege device 
				//int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);
				send_cmd_udp_f(info->fd_udp, info->lidars[i].ip, info->lidars[i].port, 
					0x4b41, rand(), sizeof(alive), &alive, false);
			}

			tto = tv.tv_sec + 3;
		}
	}
	return NULL;
}

int AddLidar(HReader hr, const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub)
{
	UDPInfo* info = (UDPInfo*)hr;

	if (info->nnode >= MAX_LIDARS) 
	{
		printf("There has %d lidars\n", info->nnode);
		return -1;
	}

	info->lidars[info->nnode].hParser = hParser;
	info->lidars[info->nnode].hPublish = hPub;
	strcpy(info->lidars[info->nnode].ip, lidar_ip);
	info->lidars[info->nnode].port = lidar_port;
	info->lidars[info->nnode].s_addr = inet_addr(lidar_ip);

	printf("add lidar [%d] %s:%d\n", info->nnode, lidar_ip, lidar_port); 

	return info->nnode++;
}

void StopUDPReader(HReader hr)
{
	UDPInfo* info = (UDPInfo*)hr;
	close(info->fd_udp);
	//printf("stop udp reader\n");
	//info->should_exit = true;
	//sleep(1);
	pthread_join(info->thr, NULL);

	delete info;
}

HReader StartUDPReader( unsigned short listen_port, bool is_group_listener, const char* group_ip, 
	int lidar_count, const LidarInfo* lidars)
{
	UDPInfo* info = new UDPInfo;
	memset(info, 0, sizeof(UDPInfo));

	for (int i=0; i<lidar_count; i++)
	{
		AddLidar(info, lidars[i].lidar_ip, lidars[i].lidar_port, lidars[i].parser, lidars[i].pub);
	}

	info->listen_port = listen_port;
	info->is_group_listener = is_group_listener;

	// open UDP port
	info->fd_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	int one = 1;
    setsockopt(info->fd_udp, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(listen_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(info->fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (rt != 0)
	{
		printf("bind port %d failed\n", listen_port);
	}
	
	printf("start udp listen port %d udp %d\n", listen_port, info->fd_udp);

	if (is_group_listener) 
	{
		ip_mreq group;
		memset(&group, 0, sizeof(group)); 
		group.imr_multiaddr.s_addr = inet_addr(group_ip);
		group.imr_interface.s_addr = INADDR_ANY; 

		int rt = setsockopt(info->fd_udp, IPPROTO_IP, 
			IP_ADD_MEMBERSHIP, (char*)&group,
			sizeof(group));
			
		printf("Adding to multicast group %s %s\n", group_ip, rt < 0 ? "fail!" : "ok");
	}

	pthread_create(&info->thr, NULL, UdpThreadProc, info); 

	return info;
} 

HReader StartUDPReader(const char* lidar_ip, unsigned short lidar_port, unsigned short local_port, 
		bool is_group_listener, const char* group_ip, 
		HParser hParser, HPublish hPub) 
{
	LidarInfo lidars[MAX_LIDARS];
	lidars[0].parser = hParser;
	lidars[0].pub = hPub;
	strcpy(lidars[0].lidar_ip, lidar_ip);
	lidars[0].lidar_port = lidar_port;
	
	return StartUDPReader( local_port, is_group_listener, group_ip, 1, lidars);
}

bool SendUdpCmd(HReader hr, int lidar_id, int len, char* cmd)
{
	UDPInfo* info = (UDPInfo*)hr;
	if (!info || info->fd_udp <= 0 || info->is_group_listener)
		return false;

	if (lidar_id < 0 || lidar_id >= info->nnode)
		return false;
		
	return send_cmd_udp(info->fd_udp, info->lidars[lidar_id].ip, info->lidars[lidar_id].port, 
		0x0043, rand(), len, cmd);
}


