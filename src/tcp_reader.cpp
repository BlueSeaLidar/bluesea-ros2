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

struct TCPInfo 
{
	HParser hParser;
	HPublish hPublish;
	bool should_exit;
	int fd_tcp;
	int lidar_port;
	char lidar_ip[32];
	//int listen_port;
	//char group_ip[32];
	pthread_t thr;
};
	
int tcp_reader(int sock, HParser hParser, HPublish hPublish)
{
	RawData* fans[MAX_FANS];

	while (1) 
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(sock, &fds); 
	
		struct timeval to = { 5, 5 }; 
		int ret = select(sock+1, &fds, NULL, NULL, &to); 

		if (ret == 0) {

		}
		
		if (ret < 0) {
			printf("select error\n");
			continue;
	       	}

		// read TCP data
		if (FD_ISSET(sock, &fds)) 
		{
			unsigned char buf[1024];
			int nr = recv(sock, buf, sizeof(buf), 0); 
			if (nr <= 0) {
				printf("read tcp error %d\n",  nr);
				break;
			} 
			else 
			{
				int nfan = ParserRunStream(hParser, nr, buf, &(fans[0]));
				if (nfan > 0) {
					PublishData(hPublish, nfan, fans);
				}
			}
		}
	}

	return 0;
}

void* TcpThreadProc(void* p)
{
	TCPInfo* info = (TCPInfo*)p;

	while (!info->should_exit) 
	{ 
		// open TCP 
		int sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	       	if (sock < 0) { 
			printf("socket TCP failed\n");
		       	return 0; 
		}

		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));     /* Zero out structure */
		addr.sin_family      = AF_INET;             /* Internet address family */

		addr.sin_addr.s_addr = inet_addr(info->lidar_ip);
		addr.sin_port = htons(info->lidar_port);

		int ret = connect(sock, (struct sockaddr *) &addr, sizeof(addr)); 
			
		if (ret != 0) 
		{
			printf("connect (%s:%d) failed", info->lidar_ip, info->lidar_port);
			close(sock); 
			//sleep(15);
			continue;
		}
		info->fd_tcp = sock;
		printf("connect (%s:%d) ok", info->lidar_ip, info->lidar_port);

		tcp_reader(sock, info->hParser, info->hPublish);

		printf("disconnect (%s:%d)\n", info->lidar_ip, info->lidar_port);

		info->fd_tcp = 0;
		close(sock);
	}
	pthread_exit(NULL);
	return NULL;
}

#if 0
void* TcpSrvProc(void* p)
{
	TCPInfo* info = (TCPInfo*)p;

	// open TCP port
	int sock  = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(info->listen_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(info->fd_tcp, (struct sockaddr *)&addr, sizeof(addr));
	
	if (rt != 0)
	{
		printf("bind port %d failed\n", info->listen_port);
		return NULL;
	}
	
	printf("start tcp server %s:%d\n", lidar_ip, lidar_port);


	while (1) {


	}

	return NULL;
}


HReader StartTCPServer(unsigned short listen_port, HParser hParser, HPublish hPub)
{
	signal(SIGPIPE, SIG_IGN);

	TCPInfo* info = new TCPInfo;
	info->hParser = hParser;
	info->hPublish = hPub;
	info->listen_port = listen_port;

	pthread_create(&info->thr, NULL, TcpSrvProc, info); 

	return info;
}
#endif

HReader StartTCPReader(const char* lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub)
{
	signal(SIGPIPE, SIG_IGN);

	TCPInfo* info = new TCPInfo;
	info->hParser = hParser;
	info->hPublish = hPub;
	info->lidar_port = lidar_port;
	strcpy(info->lidar_ip, lidar_ip);
	//strcpy(info->group_ip, group_ip);
	//printf("start udp %s:%d udp %d\n", lidar_ip, lidar_port, info->fd_tcp);

	pthread_create(&info->thr, NULL, TcpThreadProc, info); 

	return info;
} 

void StopTCPReader(HReader hr)
{
	TCPInfo* info = (TCPInfo*)hr;
	info->should_exit = true;
	//sleep(1);
	pthread_join(info->thr, NULL);
	delete info;
}

bool SendTcpCmd(HReader hr, int len, char* cmd)
{
	TCPInfo* info = (TCPInfo*)hr;
	if (!info || info->fd_tcp <= 0)
		return false;
		
	return send(info->fd_tcp, cmd, len, 0) == len;
}



