#include "alarm.h"
#include "reader.h"

void send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port, int cmd, int sn, int len, const void* snd_buf)
{
    char buffer[2048];
    CmdHeader* hdr = (CmdHeader*)buffer;
    hdr->sign = 0x484c;
    hdr->cmd = cmd;
    hdr->sn = sn;

    len = ((len + 3) >> 2) * 4;

    hdr->len = len;

    memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

    unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
    pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len / 4 + 2);

    sockaddr_in to;
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = inet_addr(dev_ip);
    to.sin_port = htons(dev_port);

    int len2 = len + sizeof(CmdHeader) + 4;

    sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

}

bool udp_talk_S_PACK(int fd_udp, const char* ip, int port, int n, const char* cmd, void* result)
{
	ROS_INFO("%s",cmd);
    unsigned short sn = rand();
    send_cmd_udp(fd_udp, ip, port, 0x0053, sn, n, cmd);

    int nr = 0;
    for (int i = 0; i < 1000; i++)
    {
        fd_set fds;
        FD_ZERO(&fds);

        FD_SET(fd_udp, &fds);

        struct timeval to = { 3, 0 };
        int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);
        if (ret <= 0)
        {
            return false;
        }
        // read UDP data
        if (FD_ISSET(fd_udp, &fds))
        {
            nr++;
            sockaddr_in addr;
            socklen_t sz = sizeof(addr);
            char buf[1024] = { 0 };
            int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
            if (nr > 0)
            {
                CmdHeader* hdr = (CmdHeader*)buf;
                if (hdr->sign != 0x484c || hdr->sn != sn)
                    continue;
                memcpy(result, buf + 8, 2);
                return true;
            }
        }
    }

    printf("read %d packets, not response\n", nr);
    return false;
}

bool udp_talk_C_PACK(int fd_udp, const char* lidar_ip, int lidar_port,
    int n, const char* cmd,
    int nhdr, const char* hdr_str,
    int nfetch, char* fetch)
{
    printf("send command : \'%s\' \n", cmd);

    unsigned short sn = rand();
    send_cmd_udp(fd_udp, lidar_ip, lidar_port, 0x0043, sn, n, cmd);

    time_t t0 = time(NULL);
    int ntry = 0;
    while (time(NULL) < t0 + 3 && ntry < 1000)
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_udp, &fds);

        struct timeval to = { 1, 0 };
        int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

        if (ret < 0)
        {
            printf("select error\n");
            return false;
        }
        if (ret == 0)
        {
            continue;
        }

        // read UDP data
        if (FD_ISSET(fd_udp, &fds))
        {
            ntry++;
            sockaddr_in addr;
            socklen_t sz = sizeof(addr);

            char buf[1024] = { 0 };
            int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
            if (nr > 0)
            {
                CmdHeader* hdr = (CmdHeader*)buf;
                if (hdr->sign != 0x484c || hdr->sn != sn)
                    continue;

                char* payload = buf + sizeof(CmdHeader);
                for (int i = 0; i < nr - nhdr - 1; i++)
                {
                    if (memcmp(payload + i, hdr_str, nhdr) == 0)
                    {
                        if (nfetch > 0)
                        {
                            memset(fetch, 0, nfetch);
                            for (int j = 0; j < nfetch && i + nhdr + j < nr; j++)
                                fetch[j] = payload[i + nhdr + j];
                        }
                        return true;
                    }
                }
            }
        }
    }
    printf("read %d packets, not response\n", ntry);
    return false;
}



void *UdpThreadProc(void *p)
{
	UDPInfo *info = (UDPInfo *)p;
	int fd_udp = info->fd_udp;
	int error_num = 0;
	char buf[1024];

	timeval tv;
	gettimeofday(&tv, NULL);

	time_t tto = tv.tv_sec + 1;
	uint32_t delay = 0;
	if (!info->is_group_listener)
	{
		for (int i = 0; i < info->nnode; i++)
		{
			setup_lidar_udp(info->lidars[i].hParser, info->fd_udp);
		}
	}
	while (1)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_udp, &fds);
		struct timeval to = {1, 5};
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret == 0)
		{
			if (!info->is_group_listener)
			{
				for (int i = 0; i < info->nnode; i++)
				{
					char cmd[12] = "LGCPSH";
					send_cmd_udp(info->fd_udp,
								 info->lidars[i].ip, info->lidars[i].port,
								 0x0043, rand(), 6, cmd);
				}
			}
		}

		if (ret < 0)
		{
			 //printf("%d select error\n",fd_udp);
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
				for (int i = 0; i < info->nnode; i++)
				{
					if (addr.sin_addr.s_addr == info->lidars[i].s_addr)
					{
						id = i;
						//saveLog(logPath, 1, info->lidars[i].ip, info->lidars[i].port, (unsigned char *)buf, sizeof(buf));
						break;
					}
				}
				if (id == -1)
				{
					// continue;
					// printf("packet from unknown address %s\n", inet_ntoa(addr.sin_addr));
				}
				else if (buf[0] == 0x4c && buf[1] == 0x48 && buf[2] == ~0x41 && buf[3] == ~0x4b)
				{
					if (nr == sizeof(KeepAlive) + 12)
					{
						uint32_t clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
						KeepAlive *ka = (KeepAlive *)(buf + 8);
						if (clock >= ka->world_clock)
							delay = clock - ka->world_clock;
						else
							delay = clock + 36000000 - ka->world_clock;
					}
				}
				else
				{
					//printf(" %d %s\n",__LINE__,__FUNCTION__);
					RawData *fans[MAX_FANS];
					int nfan = ParserRun(info->lidars[id], nr, (uint8_t *)buf, &(fans[0]));
					// test for span  without  0/180
					//for (int i=0; i<nfan; i++)
						//printf("angle:%d  span:%d time:%d %d\n", fans[i]->angle, fans[i]->span,fans[i]->ts[0],fans[i]->ts[1]);
					if (nfan > 0)
						PublishData(info->lidars[id].hPublish, nfan, fans);
				}
			}
		}

		if (tv.tv_sec > tto && !info->is_group_listener)
		{
			for (int i = 0; i < info->nnode; i++)
			{
				KeepAlive alive;
				gettimeofday(&tv, NULL);
				alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
				alive.delay = delay;

				// acknowlege device
				// int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);
				send_cmd_udp(info->fd_udp, info->lidars[i].ip, info->lidars[i].port,
							   0x4b41, rand(), sizeof(alive), &alive);
			}

			tto = tv.tv_sec + 3;
		}
	}
	
	return NULL;
}

int AddLidar(HReader hr, const char *lidar_ip, unsigned short lidar_port, HParser hParser, HPublish hPub)
{
	UDPInfo *info = (UDPInfo *)hr;

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

HReader StartUDPReader(const char *type, unsigned short listen_port, bool is_group_listener, const char *group_ip,
					   int lidar_count, const LidarInfo *lidars)
{
	UDPInfo *info = new UDPInfo;
	memset(info, 0, sizeof(UDPInfo));

	for (int i = 0; i < lidar_count; i++)
	{
		AddLidar(info, lidars[i].lidar_ip, lidars[i].lidar_port, lidars[i].parser, lidars[i].pub);
	}

	strcpy(info->type, type);
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
							IP_ADD_MEMBERSHIP, (char *)&group,
							sizeof(group));

		printf("Adding to multicast group %s %s\n", group_ip, rt < 0 ? "fail!" : "ok");
	}
	pthread_create(&info->thr, NULL, UdpThreadProc, info);
	return info;
}

bool SendUdpCmd(HReader hr, int lidar_id, int len, char *cmd)
{
	UDPInfo *info = (UDPInfo *)hr;
	if (!info || info->fd_udp <= 0 || info->is_group_listener)
		return false;

	if (lidar_id < 0 || lidar_id >= info->nnode)
		return false;

	send_cmd_udp(info->fd_udp, info->lidars[lidar_id].ip, info->lidars[lidar_id].port,0x0043, rand(), len, cmd);
	return true;
}
bool SendUdpCmd2(HReader hr, char *ip, int len, char *cmd)
{
	UDPInfo *info = (UDPInfo *)hr;
	if (!info || info->fd_udp <= 0 || info->is_group_listener)
		return false;

	for (int i = 0; i < info->nnode; i++)
	{
		if (strcmp(ip, info->lidars[i].ip) == 0)
		{
			send_cmd_udp(info->fd_udp, info->lidars[i].ip, info->lidars[i].port,
								0x0053, rand(), len, cmd);
			return true;
		}
	}
	return false;
}

bool udp_talk_GS_PACK(int fd_udp, const char *ip, int port, int n, const char *cmd, void *result)
{
	unsigned short sn = rand();
	send_cmd_udp(fd_udp, ip, port, 0x4753, sn, n, cmd);

	int nr = 0;
	for (int i = 0; i < 1000; i++)
	{
		fd_set fds;
		FD_ZERO(&fds);

		FD_SET(fd_udp, &fds);

		struct timeval to = {1, 0};
		int ret = select(fd_udp + 1, &fds, NULL, NULL, &to);

		if (ret <= 0)
		{
			return false;
		}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds))
		{
			nr++;
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = {0};
			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
				if (hdr->sign != 0x484c || hdr->sn != sn)
					continue;

				memcpy(result, buf + sizeof(CmdHeader), sizeof(EEpromV101));
				// EEpromV101 t;
				// memcpy(&t, result, sizeof(EEpromV101));
				return true;
			}
		}
	}

	printf("read %d packets, not response\n", nr);
	return false;
}

void StopUDPReader(HReader hr)
{
	UDPInfo *info = (UDPInfo *)hr;
	close(info->fd_udp);
	// printf("stop udp reader\n");
	// info->should_exit = true;
	// sleep(1);
	pthread_join(info->thr, NULL);

	delete info;
}
bool setup_lidar_udp(HParser hP, int handle)
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
		if (udp_talk_S_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.ats, result))
		{
			ROS_INFO("set ats %s\n", result);
			break;
		}
	}

	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.uuid);
		if (cmdLength <= 0)
			break;
		if (udp_talk_C_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.uuid, 11, "PRODUCT SN:", 20, buf))
		{
			std::string sn = stringfilter(buf, 20);
			ROS_INFO("uuid:%s\n", sn.c_str());
			break;
		}
		else if (udp_talk_C_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.uuid, 10, "VENDOR ID:", 20, buf))
		{
			std::string sn = stringfilter(buf, 20);
			ROS_INFO("uuid:%s\n", sn.c_str());
			break;
		}
	}

	// enable/disable shadow filter
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.fitter);
		if (cmdLength <= 0)
			break;

		if (udp_talk_S_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.fitter, result))
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

		if (udp_talk_S_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.smooth, result))
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

		if (udp_talk_S_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.rpm, result))
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

		if (udp_talk_C_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.res, 2, "OK", 0, NULL))
		{
			ROS_INFO("%s OK\n", parser->cmd.res);
			break;
		}
	}

	// enable/disable alaram message uploading
	for (unsigned int i = 0; i < index; i++)
	{
		cmdLength = strlen(parser->cmd.alarm);
		if (cmdLength <= 0)
			break;
		if (udp_talk_S_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.alarm, result))
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
		if (udp_talk_S_PACK(handle, parser->ip, parser->port, cmdLength, parser->cmd.direction, result))
		{
			ROS_INFO("set direction %s\n", result);
			break;
		}
	}

	return true;
}
