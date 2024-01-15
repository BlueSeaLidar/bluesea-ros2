#include "reader.h"


extern "C" int change_baud(int fd, int baud);

bool uart_talk(int fd, int n, const char *cmd, int nhdr, const char *hdr_str, int nfetch, char *fetch, int waittime,int CacheLength)
{
	//printf("send command : %s\n", cmd);
	write(fd, cmd, n);

	char *buf=new char[CacheLength];
	int nr = read(fd, buf, sizeof(buf));
	int idx = waittime;

	while (nr < CacheLength)
	{
		int n = read(fd, buf + nr, CacheLength - nr);
		// printf(" fd %d %d \n",n,nr);
		if (n > 0)
		{
			nr += n;
			idx = waittime;
		}
		else if (n == 0)
		{
			idx--;
			usleep(1000);
			if (idx == 0)
			{
				// printf("read 0 byte max index break\n");
				break;
			}
		}
	}
	// if(idx>0)
	//     printf("read max byte break\n");
	bool res=false;
	for (unsigned int i = 0; i < CacheLength - nhdr - nfetch; i++)
	{
		if (memcmp(buf + i, hdr_str, nhdr) == 0)
		{
			if (nfetch > 0)
			{
				if (strcmp(cmd, "LXVERH") == 0 || strcmp(cmd, "LVERSH") == 0 || strcmp(cmd, "LUUIDH") == 0 || strcmp(cmd, "LTYPEH") == 0 || strcmp(cmd, "LQAZNH") == 0 || strcmp(cmd, "LQPSTH") == 0 || strcmp(cmd, "LQNPNH") == 0 || strcmp(cmd, "LQOUTH") == 0 || strcmp(cmd, "LQCIRH") == 0 || strcmp(cmd, "LQFIRH") == 0 || strcmp(cmd, "LQSRPMH") == 0 || strcmp(cmd, "LQSMTH") == 0 || strcmp(cmd, "LQDSWH") == 0 || strcmp(cmd, "LQZTPH") == 0)
				{
					int length =CacheLength - nhdr - i;
					if(length>nfetch)
						length=nfetch;
					
					memcpy(fetch, buf + i + nhdr, length);
					fetch[length] = 0;
				}
				else if (strstr(cmd, "LSRPM") != NULL)
				{
					if (buf[i + nhdr + 1] == 'O' && buf[i + nhdr + 2] == 'K')
					{
						strncpy(fetch, "OK", 2);
						fetch[2] = 0;
					}
					else if (buf[i + nhdr + 1] == 'e' && buf[i + nhdr + 2] == 'r')
					{
						strncpy(fetch, "NG", 2);
						fetch[2] = 0;
					}
				}
				else
				{
					strncpy(fetch, "OK", 2);
					fetch[2] = 0;
				}
			}
			delete []buf;
			return true;
		}
		else if (memcmp(buf + i, cmd, n) == 0)
		{
			if (nfetch > 0)
			{
				memcpy(fetch, buf + i + n + 1, 2);
				if (buf[i + n + 1] == 'E' && buf[i + n + 2] == 'R')
				{
					fetch[0] = 'N';
					fetch[1] = 'G';
				}
				fetch[2] = 0;
			}
			delete []buf;
			return true;
		}
		else if (memcmp(buf + i, "unsupport", 9) == 0)
		{
			//ROS_INFO("%s %d \n", __FUNCTION__, __LINE__);
			if (nfetch > 0)
			{
				strcpy(fetch, "unsupport");
				fetch[10] = 0;
			}
			delete []buf;
			return true;
		}
	}
	delete []buf;
	ROS_ERROR("read %d bytes, not found %s\n", nr, hdr_str);
	return false;
}

bool vpc_talk(int hcom, int mode, short sn, int len, const char *cmd, int nfetch, void *result)
{
	printf("USB send command : %s\n", cmd);
	char buffer[2048];
	CmdHeader *hdr = (CmdHeader *)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = mode;
	hdr->sn = sn;
	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), cmd, len);

	unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);
	// pcrc[0] = BaseAPI::stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

	int len2 = len + sizeof(CmdHeader) + 4;
	int nr = write(hcom, buffer, len2);
	char buf[2048];
	int index = 10;
	// 4C 48 BC FF   xx xx xx xx  result
	// 读取之后的10*2048个长度，如果不存在即判定失败
	while (index--)
	{
		nr = read(hcom, buf, sizeof(buf));
		while (nr < (int)sizeof(buf))
		{
			int n = read(hcom, buf + nr, sizeof(buf) - nr);
			if (n > 0)
				nr += n;
		}

		for (int i = 0; i < (int)sizeof(buf) - nfetch; i++)
		{
			if (mode == C_PACK)
			{
				char *fetch = (char *)result;
				if (buf[i] == 0x4C && buf[i + 1] == 0x48 && buf[i + 2] == (signed char)0xBC && buf[i + 3] == (signed char)0xFF)
				{
					/*int packSN = ((unsigned int)buf[i + 5] << 8) | (unsigned int)buf[i + 4];
					if (packSN != sn)
						continue;*/

					for (int j = 0; j < nfetch; j++)
					{
						if ((buf[i + j + 8] >= 33 && buf[i + j + 8] <= 127))
						{
							fetch[j] = buf[i + j + 8];
						}
						else
						{
							fetch[j] = ' ';
						}
					}
					fetch[nfetch] = 0;
					return true;
				}
			}
			else if (mode == S_PACK)
			{
				if ((buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xB8) || (buf[i + 2] == (signed char)0xAC && buf[i + 3] == (signed char)0xff))
				{
					// printf("%02x  %02x\n", buf[i + 2], buf[i + 3]);
					// 随机码判定
					short packSN = ((unsigned char)buf[i + 5] << 8) | (unsigned char)buf[i + 4];
					if (packSN != sn)
						continue;

					memcpy(result, buf + i + 8, nfetch);
					return true;
				}
			}
		}
	}
	printf("read %d bytes, not found %s\n", nr, cmd);
	return false;
}

int open_serial_port(const char *port, int baudrate)
{
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		ROS_ERROR("open %s error", port);
		return -1;
	}

	int ret;
	struct termios attrs;

	tcflush(fd, TCIOFLUSH);

	/* get current attrs */
	ret = tcgetattr(fd, &attrs);
	if (ret < 0)
	{
		ROS_ERROR("get attrs failed");
		return -1;
	}

	/* set speed */
	int speed = B230400;
	// if (baudrate == 115200) speed = B115200;

	ret = cfsetispeed(&attrs, speed);  //[baudrate]);
	ret |= cfsetospeed(&attrs, speed); //[baudrate]);

	/* enable recieve and set as local line */
	attrs.c_cflag |= (CLOCAL | CREAD);

	/* set data bits */
	attrs.c_cflag &= ~CSIZE;
	attrs.c_cflag |= CS8;

	/* set parity */
	if (1)
	{							  // parity == UART_POFF) {
		attrs.c_cflag &= ~PARENB; // disable parity
		attrs.c_iflag &= ~INPCK;
	}
	else
	{
		attrs.c_cflag |= (PARENB | PARODD); // enable parity
		attrs.c_iflag |= INPCK;
		// if(parity == UART_PEVEN) attrs.c_cflag &= ~PARODD;
	}

	/* set stop bits */
	attrs.c_cflag &= ~CSTOPB; // 1 stop bit
							  // attrs.c_cflag |= CSTOPB;	// 2 stop bits

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
	if (tcsetattr(fd, TCSANOW, &attrs) < 0)
	{
		close(fd);
		ROS_ERROR("tcsetattr err");
		return -1;
	}

	if (change_baud(fd, baudrate))
	{
		close(fd);
		ROS_ERROR("fail to set baudrate %d", baudrate);
		return -1;
	}

	return fd;
}

bool setup_lidar_uart(HParser hP, int handle)
{
	Parser *parser = (Parser *)hP;
	unsigned int index = 5;
	int cmdLength;
	char buf[32];
	char result[3] = {0};
	result[2] = '\0';

	for (unsigned int i = 0; i < index; i++)
	{
		if ((uart_talk(handle, 6, "LVERSH", 14, "MOTOR VERSION:", 16, buf)))
		{
			std::string sn = stringfilter(buf, 16);
			ROS_INFO("version:%s\n", sn.c_str());
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.uuid);
		if (cmdLength <= 0)
			break;
		if ((uart_talk(handle, 6, "LUUIDH", 11, "PRODUCT SN:", 20, buf)))
		{
			std::string sn = stringfilter(buf, 20);
			ROS_INFO("uuid:%s\n", sn.c_str());
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.model);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.model, 8, "TYPE ID:", 16, buf))
		{
			std::string sn = stringfilter(buf, 16);
			ROS_INFO("product model:%s\n", sn.c_str());
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.unit_mm);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.unit_mm, 10, "SET LiDAR ", 12, buf))
		{
			ROS_INFO("set LiDAR unit %s\n",buf);
			break;
		}
	}
	// enable/disable output intensity
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.confidence);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.confidence, 6, "LiDAR ", 12, buf))
		{
			ROS_INFO("set LiDAR confidence %s\n", buf);
			break;
		}
	}
	// enable/disable shadow filter
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.fitter);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.fitter,  2, "ok", 2, buf,100))
		{
			ROS_INFO("set LiDAR shadow filter ok\n");
			break;
		}
	}
	// enable/disable smooth

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.smooth);
		if (cmdLength <= 0)
			break;

		if (uart_talk(handle, cmdLength, parser->cmd.smooth, 2, "ok", 2, buf,100))
		{
			ROS_INFO("set LiDAR smooth OK\n");
			break;
		}
	}
	// setup rpm
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.rpm);
		if (cmdLength <= 0)
			break;

		if (uart_talk(handle, cmdLength, parser->cmd.rpm, 8, "Set RPM:", 12, buf,100))
		{
			ROS_INFO("%s %s\n", parser->cmd.rpm, buf);
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.res);
		if (cmdLength <= 0)
			break;
		if (uart_talk(handle, cmdLength, parser->cmd.res, 15, "set resolution ", 12, buf))
		{
			ROS_INFO("%s %s\n", parser->cmd.res,buf);
			break;
		}
	}
	return true;
}

bool setup_lidar_vpc(HParser hP, int handle)
{
	Parser *parser = (Parser *)hP;
	unsigned int index = 5;
	int cmdLength;
	char buf[32];
	char result[3] = {0};
	result[2] = '\0';

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.ats);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0053, rand(),cmdLength, parser->cmd.ats, 2, result))
		{
			ROS_INFO("%s %s\n",  parser->cmd.ats,result);
			break;
		}
	}
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.uuid);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0043, rand(),cmdLength, parser->cmd.uuid, 20, buf))
		{
			std::string sn = stringfilter(buf, 20);
			ROS_INFO("uuid:%s\n", sn.c_str());
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.model);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0043, rand(),cmdLength, parser->cmd.model, 16, buf))
		{
			std::string sn = stringfilter(buf, 16);
			ROS_INFO("product model:%s\n", sn.c_str());
			break;
		}
	}

	// enable/disable shadow filter
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.fitter);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(),cmdLength, parser->cmd.fitter, 2, result))
		{
			ROS_INFO("set LiDAR shadow filter %s %s\n", parser->cmd.fitter, result);
			break;
		}
	}
	// enable/disable smooth
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.smooth);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053,rand(), cmdLength, parser->cmd.smooth, 2, result))
		{
			ROS_INFO("set LiDAR smooth  %s %s\n", parser->cmd.smooth, result);
			break;
		}
	}
	// setup rpm
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.rpm);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(),cmdLength, parser->cmd.rpm, 2, result))
		{
			ROS_INFO("%s %s\n", parser->cmd.rpm, result);
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.res);
		if (cmdLength <= 0)
			break;

		if (vpc_talk(handle, 0x0053, rand(),cmdLength, parser->cmd.res, 2, result))
		{
			ROS_INFO("%s %s\n", parser->cmd.res, result);
			break;
		}
	}

	// enable/disable alaram message uploading
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.alarm);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0053,  rand(),cmdLength, parser->cmd.alarm, 2, result))
		{
			ROS_INFO("set alarm_msg %s\n", result);
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.direction);
		if (cmdLength <= 0)
			break;
		if (vpc_talk(handle, 0x0053, rand(), cmdLength, parser->cmd.direction, 2, result))
		{
			ROS_INFO("set direction %s\n", result);
			break;
		}
	}
	return true;
}
int UartReader(UartInfo *info)
{
	int fd_uart = info->fd_uart;

	RawData *fans[MAX_FANS];
	while (1)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_uart, &fds);

		struct timeval to = {1, 500000};

		int ret = select(fd_uart + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			printf("select error\n");
			break;
		}

		// read UART data
		if (FD_ISSET(fd_uart, &fds))
		{
			unsigned char buf[512];
			unsigned int nr = read(fd_uart, buf, sizeof(buf));
			if (nr <= 0)
			{
				printf("read port error %d\n", nr);
				break;
			}
			// saveLog(info->logPath, 1, "UART", 0, (unsigned char *)buf, nr);
			int nfan = ParserRunStream(info->hParser, nr, buf, &(fans[0]));
			// for (int i=0; i<nfan; i++)
			// printf("angle:%d distance:%d \n", fans[i]->angle, fans[i]->points[0].distance);
			if (nfan > 0)
			{
				PublishData(info->hPublish, nfan, fans);
			}
			if (nr < sizeof(buf) - 10)
				usleep(10000 - nr * 10);
		}
	}
	return 0;
}

void *UartThreadProc(void *p)
{
	UartInfo *info = (UartInfo *)p;
	if (access(info->port, R_OK))
	{
		printf("port %s not ready\n", info->port);
		// sleep(10);
		return NULL;
	}
	int fd = open_serial_port(info->port, info->baudrate);
	if (fd > 0)
	{
		info->fd_uart = fd;
		if (strcmp(info->type, "uart") == 0)
			setup_lidar_uart(info->hParser, info->fd_uart);
		if (strcmp(info->type, "vpc") == 0)
			setup_lidar_vpc(info->hParser, info->fd_uart);

		UartReader(info);
	}

	return NULL;
}

void *StartUartReader(const char *type, const char *port, int baudrate, HParser hParser, HPublish hPublish)
{
	UartInfo *info = new UartInfo;
	strcpy(info->type, type);
	strcpy(info->port, port);
	info->baudrate = baudrate;
	info->hParser = hParser;
	info->hPublish = hPublish;
	pthread_create(&info->thr, NULL, UartThreadProc, info);

	return info;
}

bool SendUartCmd(HReader hr, int len, char *cmd)
{
	UartInfo *info = (UartInfo *)hr;
	if (info && info->fd_uart > 0)
		write(info->fd_uart, cmd, len);
	return true;
}
bool SendVpcCmd(HReader hr, int len, char *cmd)
{
	UartInfo *info = (UartInfo *)hr;
	if (info && info->fd_uart > 0)
	{
		int fd = info->fd_uart;

		unsigned char buffer[2048];
		CmdHeader *hdr = (CmdHeader *)buffer;
		hdr->sign = 0x484c;
		hdr->cmd = 0x0043;
		hdr->sn = rand();

		len = ((len + 3) >> 2) * 4;

		hdr->len = len;

		memcpy(buffer + sizeof(CmdHeader), cmd, len);

		// int n = sizeof(CmdHeader);
		unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + len);
		pcrc[0] = stm32crc((unsigned int *)(buffer + 0), len / 4 + 2);
		int len2 = len + sizeof(CmdHeader) + 4;
		write(fd, buffer, len2);
	}
	return true;
}
void StopUartReader(HReader hr)
{
	UartInfo *info = (UartInfo *)hr;
	// info->should_exit = true;
	// sleep(1);
	pthread_join(info->thr, NULL);
	delete info;
}