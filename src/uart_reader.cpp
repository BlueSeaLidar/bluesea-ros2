
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


#define ROS_ERROR printf
#define ROS_INFO printf

struct UartInfo 
{
	int fd_uart;
	char port[128];
	int baudrate;
	int* rate_list;

	bool should_exit;

	HParser hParser;
	HPublish hPublish;
	pthread_t thr;
};


extern "C" int change_baud(int fd, int baud);

#if 0
// send pacecat command to lidar serial port
int send_cmd_uart(int fd_uart, unsigned short cmd_id, int len, const void* cmd_body)
{
	char buffer[2048] = {0};
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd_id;
	static unsigned short sn = 1;
	hdr->sn = sn++;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd_body, len);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len/4 + 2);

	return write(fd_uart, buffer, len + sizeof(CmdHeader) + 4);
}
#endif

int open_serial_port(const char* port, int baudrate) 
{
       	int fd = open(port,  O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) { 
		ROS_ERROR("open %s error", port); 
		return -1; 
       	}
	
	int   		ret; 
	struct termios	attrs;

	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if(ret < 0) {
		ROS_ERROR("get attrs failed");
		return -1;
	}

	/* set speed */
	int speed = B230400;
	//if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed);//[baudrate]);  
	ret |= cfsetospeed(&attrs, speed);//[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1) {//parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB;			//disable parity
	       	attrs.c_iflag &= ~INPCK;
	} else {
		attrs.c_cflag |= (PARENB | PARODD);	//enable parity
	       	attrs.c_iflag |= INPCK;
		//if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB;	// 1 stop bit
	//attrs.c_cflag |= CSTOPB;	// 2 stop bits

	// Disable Hardware flowcontrol
        attrs.c_cflag &= ~CRTSCTS;

	/* set to raw mode, disable echo, signals */
	attrs.c_lflag &= ~(ICANON | ECHO | ECHOE | IEXTEN | ISIG);

	/* set no output process, raw mode */
	attrs.c_oflag &= ~OPOST;
	attrs.c_oflag &= ~(ONLCR | OCRNL);

	/* disable CR map  */
	attrs.c_iflag &= ~(ICRNL | INLCR);
	/* disable software flow control */
	attrs.c_iflag &= ~(IXON | IXOFF | IXANY);

	attrs.c_cc[VMIN] = 0;
	attrs.c_cc[VTIME] = 0;

	/* flush driver buf */
	tcflush(fd, TCIFLUSH);

	/* update attrs now */
	if(tcsetattr(fd, TCSANOW, &attrs) < 0) 
	{
		close(fd);
	       	ROS_ERROR("tcsetattr err");
	       	return -1;
	}

	if ( change_baud(fd, baudrate) )
	{
		close(fd);
	       	ROS_ERROR("fail to set baudrate %d", baudrate);
	       	return -1;
	}

	return fd;
}

bool quirk_talk(void* hnd,
	       	int n, const char* cmd, 
		int nhdr, const char* hdr_str, 
		int nfetch, char* fetch)
{
	UartInfo* info = (UartInfo*)hnd;
	int fd = info->fd_uart;

	printf("send command : \'%s\' \n", cmd);
	write(fd, cmd, n);
			
	char buf[4096*2];

	int nr = read(fd, buf, sizeof(buf));

	for (int i=0; i<20 && nr < (int)sizeof(buf); i++)
	{
		int sz = sizeof(buf)-nr;
		int n = read(fd, buf+nr, sz > 256 ? 256 : sz); 
		if (n > 0)
		       	nr += n;
		else 
			usleep(100*1000);
			//ros::Duration(0.1).sleep();
	}

	for (int i=0; i<nr-nhdr-nfetch; i++) 
	{
		if (memcmp(buf+i, hdr_str, nhdr) == 0) 
		{
			memcpy(fetch, buf+i+nhdr, nfetch);
			fetch[nfetch] = 0;
			return true;
		}
	}


#if 0
	char path[256];
	sprintf(path, "/tmp/%s.dat", hdr_str);
	FILE* fp = fopen(path, "wb");
	if (fp) {
		fwrite(buf, 1, nr, fp);
		fclose(fp);
	}
#endif

	printf("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}


int UartReader(UartInfo* info)
{
	int fd_uart = info->fd_uart;

	RawData* fans[MAX_FANS];

	while (!info->should_exit) 
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_uart, &fds); 
	
		struct timeval to = { 5, 1 };
		int ret = select(fd_uart+1, &fds, NULL, NULL, &to); 

		if (ret < 0) {
			printf("select error\n");
			break;
	       	}

		// read UART data
		if (FD_ISSET(fd_uart, &fds)) 
		{
			unsigned char buf[256];
			int nr = read(fd_uart, buf, sizeof(buf)); 
			if (nr <= 0) {
				printf("read port error %d\n",  nr);
				break;
			} else {
				
				int nfan = ParserRunStream(info->hParser, nr, buf, &(fans[0]));
				//printf("run stream %d\n", nfan);
				//for (int i=0; i<nfan; i++)
				//	 printf("fan %x %d + %d\n", fans[i], fans[i]->angle, fans[i]->span);
				if (nfan > 0) {
					PublishData(info->hPublish, nfan, fans);
				}
			}
		}
	}

	return 0;
}

int search_feature(int len, unsigned char* buf)
{
	int cnt = 0;
	for (int i=0; i<len-4; i++) {
		if (buf[i+1] == 0xfa) {
			if (buf[i] == 0xce || buf[i] == 0xcf || buf[i] == 0xdf)
				cnt++;
		}
	}
	return cnt;
}


int try_serial_port(const char* port, int baud_rate) 
{
	int fd = open_serial_port(port, baud_rate);
	if (fd < 0) {
		return -1;
	}

	unsigned char* buf = new unsigned char[4096];

	int nr = 0;
	time_t t = time(NULL);
	while (nr < 4096) 
	{
		int n = read(fd, buf+nr, 4096-nr);
		if (n > 0) nr += n;
		if (time(NULL) > t+1) break;
	}
	close(fd);

	int fnd = -1;
	if (nr > 1024) fnd = search_feature(nr, buf);
	delete buf;
	return fnd;
}

int detect_baudrate(const char* port, int* possible_rates)
{
	int mx = -1, idx = -1;
	for (int i=0; ; i++) 
	{
		if (possible_rates[i] == 0)
			break;

		int cnt = try_serial_port(port, possible_rates[i]);
		if (cnt > mx)
		{
			mx = cnt;
			idx = i;
		}
		//printf("%d get %d\n", possible_rates[i], cnt);
	}
	if (idx != -1) 
		return possible_rates[idx]; 
       	return -1;
}

void* UartThreadProc(void* p)
{
	UartInfo* info = (UartInfo*)p;

	while (!info->should_exit) 
	{
		if (access(info->port, R_OK)) 
		{
			printf("port %s not ready\n", info->port);
			sleep(10);
			continue;
		}
		
		//int fd = open_serial_port(info->port, info->baudrate);
		int baudrate = info->baudrate;
		if (baudrate == -1) { 
			baudrate = detect_baudrate(info->port, info->rate_list);
		}

		if (baudrate <= 0) {
			sleep(10);
			continue;
		}

		int fd = open_serial_port(info->port, baudrate);

		if (fd > 0) {
			info->fd_uart = fd;
	
			ParserScript(info->hParser, quirk_talk, info);

			UartReader(info);
		} else {
			for (int i=0; i<10 && !info->should_exit; i++) 
				sleep(1);
		}
	}
	pthread_exit(NULL);

	return NULL;
}

void* StartUartReader(const char* port, int baudrate, int* rate_list, HParser hParser, HPublish hPublish)
{
	UartInfo* info = new UartInfo;

	strcpy(info->port, port);
	info->baudrate = baudrate;
	info->rate_list = rate_list;
	info->hParser = hParser;
	info->hPublish = hPublish;
	info->should_exit = false;

	pthread_create(&info->thr, NULL, UartThreadProc, info); 

	return info;
}

void StopUartReader(HReader hr)
{
	UartInfo* info = (UartInfo*)hr;
	info->should_exit = true;
	//sleep(1);
	pthread_join(info->thr, NULL);
	delete info;
}


bool SendUartCmd(HReader hr, int len, char* cmd)
{
	UartInfo* info = (UartInfo*)hr;
	if (info && info->fd_uart > 0)
	       	write(info->fd_uart, cmd, len);
	return true;
}


