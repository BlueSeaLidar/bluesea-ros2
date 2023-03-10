#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include "parser.h"
#include "alarm.h"

#if 0
short LidarAng2ROS(short ang)
{
	//ang = -ang;
	//return ang < -1800 ? ang + 3600 : ang;
	return ang >= 1800 ? ang - 3600 : ang;
}
#endif
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

static uint32_t update_flags(unsigned char *buf)
{
	uint32_t v;
	memcpy(&v, buf, 4);
	return v;
}

static unsigned char is_data(unsigned char *buf)
{
	if (buf[1] != 0xFA)
		return 0;

	if (buf[0] == 0xCE || buf[0] == 0xCF || buf[0] == 0xDF || buf[0] == 0xC7)
		return buf[0];

	return 0;
}

static RawData *GetData0xCE_2(const RawDataHdr &hdr, unsigned char *buf, uint32_t flags, bool with_chk)
{
	RawData *dat = new RawData;
	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	memcpy(dat, &hdr, HDR_SIZE);
	dat->span = 360;

	int is_mm = flags & DF_UNIT_IS_MM;
	int with_conf = flags & DF_WITH_INTENSITY;

	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	unsigned char *pdat = buf + HDR_SIZE;
	for (int i = 0; i < hdr.N; i++)
	{
		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short val = (v2 << 8) | v;

		if (with_conf)
		{
			dat->points[i].confidence = val >> 13;
			dat->points[i].distance = val & 0x1fff;
			if (is_mm == 0)
				dat->points[i].distance *= 10;
		}
		else
		{
			dat->points[i].distance = is_mm ? val : val * 10;
			dat->points[i].confidence = 0;
		}

		// dat->points[i].angle = hdr.angle*10 + 3600 * i / hdr.N;
		dat->points[i].degree = hdr.angle / 10.0 + (dat->span * i) / (10.0 * hdr.N);

		sum += val;
	}

	memcpy(&chk, buf + HDR_SIZE + hdr.N * 2, 2);

	if (with_chk && chk != sum)
	{
		delete dat;
		printf("chksum error");
		return NULL;
	}

	// dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);

	SetTimeStamp(dat);

	return dat;
}

static RawData *GetData0xCE_3(const RawDataHdr &hdr, unsigned char *buf, uint32_t flags, bool with_chk)
{
	RawData *dat = new RawData;
	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	memcpy(dat, &hdr, HDR_SIZE);
	int span = (flags & DF_FAN_90) ? 90 : 360;

	if (hdr.angle == 3420 && (flags & DF_FAN_90))
		span = 180;
	dat->span = span;

	unsigned char *pdat = buf + HDR_SIZE;
	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	for (int i = 0; i < hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2 << 8) | v;
		sum += vv;
		dat->points[i].distance = vv;
		// dat->points[i].angle = hdr.angle*10 + span * i * 10 / hdr.N;
		dat->points[i].degree = hdr.angle / 10.0 + (span * i) / (10.0 * hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk && chk != sum)
	{
		delete dat;
		printf("chksum ce error");
		return NULL;
	}

	// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	// printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	SetTimeStamp(dat);
	// dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static FanSegment *GetFanSegment(const RawDataHdr7 &hdr, uint8_t *pdat, bool /*with_chk*/)
{
	FanSegment *fan_seg = new FanSegment;
	if (!fan_seg)
	{
		printf("out of memory\n");
		return NULL;
	}
	fan_seg->hdr = hdr;
	fan_seg->next = NULL;

	uint16_t sum = 0;
	// if (with_chk)
	{
		uint16_t *pchk = (uint16_t *)pdat;
		for (int i = 1; i < HDR7_SIZE / 2; i++)
			sum += pchk[i];
	}

	uint8_t *pDist = pdat + HDR7_SIZE;
	uint8_t *pAngle = pdat + HDR7_SIZE + 2 * hdr.N;
	uint8_t *energy = pdat + HDR7_SIZE + 4 * hdr.N;

	for (int i = 0; i < hdr.N; i++, pDist += 2, pAngle += 2)
	{
		fan_seg->dist[i] = ((uint16_t)(pDist[1]) << 8) | pDist[0];
		fan_seg->angle[i] = ((uint16_t)(pAngle[1]) << 8) | pAngle[0];
		fan_seg->energy[i] = energy[i];

		sum += fan_seg->dist[i];
		sum += fan_seg->angle[i];
		sum += energy[i];
	}

	uint8_t *pchk = pdat + HDR7_SIZE + 5 * hdr.N;
	uint16_t chksum = ((uint16_t)(pchk[1]) << 8) | pchk[0];
	if (chksum != sum)
	{
		printf("checksum error\n");
		delete fan_seg;
		return NULL;
	}

	return fan_seg;
}

void DecTimestamp(uint32_t ts, uint32_t *ts2)
{
	timeval tv;
	gettimeofday(&tv, NULL);
	uint32_t sec = tv.tv_sec % 3600;
	if (sec < 5 && ts / 1000 > 3595)
	{
		ts2[0] = (tv.tv_sec / 3600 - 1) * 3600 + ts / 1000;
	}
	else
	{
		ts2[0] = (tv.tv_sec / 3600) * 3600 + ts / 1000;
	}

	ts2[1] = (ts % 1000) * 1000000;
}

static RawData *PackFanData(FanSegment *seg)
{
	RawData *dat = new RawData;
	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	dat->code = 0xfac7;
	dat->N = seg->hdr.whole_fan;
	dat->angle = seg->hdr.beg_ang / 100;					 // 0.1 degree
	dat->span = (seg->hdr.end_ang - seg->hdr.beg_ang) / 100; // 0.1 degree
	dat->fbase = 0;
	dat->first = 0;
	dat->last = 0;
	dat->fend = 0;
	dat->counterclockwise = 0;

	DecTimestamp(seg->hdr.timestamp, dat->ts);
	int count = 0;
	while (seg)
	{
		for (int i = 0; i < seg->hdr.N; i++, count++)
		{
			dat->points[count].confidence = seg->energy[i];
			dat->points[count].distance = seg->dist[i];
			dat->points[count].degree = (seg->angle[i] + seg->hdr.beg_ang) / 1000.0;
		}

		seg = seg->next;
	}

	return dat;
}

static int GetFanPointCount(FanSegment *seg)
{
	int n = 0;

	while (seg)
	{
		n += seg->hdr.N;
		seg = seg->next;
	}

	return n;
}

static RawData *GetData0xC7(Parser *parser, const RawDataHdr7 &hdr, uint8_t *pdat)
{
	if (parser->dev_id != (u_int32_t)ANYONE && hdr.dev_id != parser->dev_id)
	{
		static time_t last = 0;
		time_t t = time(NULL);
		if (t > last)
		{
			printf("device id [%d] != my id [%d]\n", hdr.dev_id, parser->dev_id);
			last = t;
		}
		// not my data
		return NULL;
	}

	FanSegment *fan_seg = GetFanSegment(hdr, pdat, parser->with_chk);
	if (!fan_seg)
	{
		return NULL;
	}

	// printf("fan %d %d\n", fan_seg->hdr.beg_ang, fan_seg->hdr.ofset);

	if (parser->fan_segs != NULL)
	{
		FanSegment *seg = parser->fan_segs;

		if (seg->hdr.timestamp != fan_seg->hdr.timestamp)
		{
			printf("drop old fan segments\n");
			while (seg)
			{
				parser->fan_segs = seg->next;
				delete seg;
				seg = parser->fan_segs;
			}
			parser->fan_segs = fan_seg;
		}
		else
		{
			while (seg)
			{
				if (seg->hdr.ofset == fan_seg->hdr.ofset)
				{
					printf("drop duplicated segment\n");
					delete fan_seg;
					fan_seg = NULL;
					break;
				}
				if (seg->next == NULL)
				{
					seg->next = fan_seg;
					break;
				}
				seg = seg->next;
			}
		}
	}

	if (parser->fan_segs == NULL && fan_seg != NULL)
	{
		parser->fan_segs = fan_seg;
	}

	// if (parser->fan_segs == NULL) { return NULL; }

	unsigned int N = GetFanPointCount(parser->fan_segs);

	if (N >= parser->fan_segs->hdr.whole_fan)
	{
		RawData *dat = NULL;

		if (N == parser->fan_segs->hdr.whole_fan)
		{
			if (N > sizeof(dat->points) / sizeof(dat->points[0]))
			{
				printf("too many %d points in 1 fan\n", N);
			}
			else
			{
				dat = PackFanData(parser->fan_segs);
			}
		}

		// remove segments
		FanSegment *seg = parser->fan_segs;
		while (seg)
		{
			parser->fan_segs = seg->next;
			delete seg;
			seg = parser->fan_segs;
		}

		if (dat)
		{
			// SetTimeStamp(dat, );
			// dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
		}

		return dat;
	}

	return NULL;
}

static RawData *GetData0x99(const RawDataHdr99 &hdr, unsigned char *pdat, bool)
{
	RawData *dat = new RawData;

	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}

	memset(dat, 0, sizeof(RawData));
	dat->code = hdr.code;
	dat->N = hdr.N;
	dat->angle = hdr.from * 3600 / hdr.total; // 0.1 degree
	dat->span = hdr.N * 3600 / hdr.total;	  // 0.1 degree
	// dat->fbase = ;
	// dat->first;
	// dat->last;
	// dat->fend;
	DecTimestamp(hdr.timestamp, dat->ts);

	dat->counterclockwise = (hdr.flags & DF_MOTOR_REVERSE) ? 1 : 0;

	pdat += HDR99_SIZE;

	uint8_t *dist = pdat;
	uint8_t *energy = pdat + 2 * hdr.N;

	for (int i = 0; i < hdr.N; i++)
	{
		uint16_t lo = *dist++;
		uint16_t hi = *dist++;
		dat->points[i].distance = (hi << 8) | lo;
		dat->points[i].degree = (i + hdr.from) * 360.0 / hdr.total;
		dat->points[i].confidence = energy[i];
	}

	return dat;
}

static RawData *GetData0xCF(const RawDataHdr2 &hdr, unsigned char *pdat, bool with_chk)
{
	RawData *dat = new RawData;
	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	memcpy(dat, &hdr, HDR2_SIZE);

	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	pdat += HDR2_SIZE;

	for (int i = 0; i < hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;

		unsigned short vv = (v2 << 8) | v;

		sum += vv;
		dat->points[i].distance = vv;
		// dat->points[i].angle = hdr.angle*10 + hdr.span * i *10 / hdr.N;
		dat->points[i].degree = hdr.angle / 10.0 + (hdr.span * i) / (10.0 * hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk && chk != sum)
	{
		delete dat;
		printf("chksum cf error");
		return NULL;
	}

	// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	// printf("get CF %d(%d) %d\n", hdr.angle, hdr.N, hdr.span);

	SetTimeStamp(dat);
	// dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static RawData *GetData0xDF(const RawDataHdr3 &hdr, unsigned char *pdat, bool with_chk)
{
	RawData *dat = new RawData;

	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	memcpy(dat, &hdr, HDR3_SIZE);

	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	sum += hdr.fbase;
	sum += hdr.first;
	sum += hdr.last;
	sum += hdr.fend;

	double dan = (hdr.last - hdr.first) / double(hdr.N - 1);

	pdat += HDR3_SIZE;

	for (int i = 0; i < hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2 << 8) | v;
		sum += vv;
		dat->points[i].distance = vv;
		// dat->points[i].angle = hdr.first + dan * i;
		dat->points[i].degree = (hdr.first + dan * i) / 100.0;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk && chk != sum)
	{
		delete dat;
		printf("chksum df error");
		return NULL;
	}

	// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	// printf("get DF %d=%d %d %d\n", hdr.angle, hdr.first, hdr.N, hdr.span);

	SetTimeStamp(dat);
	// dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static int MsgProc(Parser *parser, int len, unsigned char *buf)
{
	if (len >= 8 && buf[0] == 'S' && buf[1] == 'T' && buf[6] == 'E' && buf[7] == 'D')
	{
		parser->flags = update_flags(buf + 2);
		return 0;
	}
	else if (alarmProc(buf, len))
	{
		return 0;
	}
	else
	{
		printf("unknown %d bytes : %02x ", len, buf[0]);
		for (int i = 1; i < len && i < 16; i++)
			printf("%02x ", buf[i]);
		printf("\n");
	}

	return -1;
}

static int ParseStream(Parser *parser, int len, unsigned char *buf, int *nfan, RawData *fans[])
{
	int max_fan = *nfan;
	int idx = 0;
	*nfan = 0;

	int unk = 0;
	unsigned char unknown[512];

	while (idx < len - 32 && *nfan < max_fan)
	{
		unsigned char type = is_data(buf + idx);
		if (type == 0)
		{
			unknown[unk++] = buf[idx++];
			if (unk == sizeof(unknown))
			{
				printf("drop %d bytes : %02x %02x %02x %02x ...\n", unk,
					   unknown[0], unknown[1],
					   unknown[2], unknown[3]);
				unk = 0;
			}
			continue;
		}

		if (unk > 0)
		{
			MsgProc(parser, unk, unknown);
			unk = 0;
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf + idx, HDR_SIZE);

		if (hdr.N > MAX_POINTS || hdr.N < 10)
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		if (type == 0xCE)
		{
			if (parser->raw_mode == 2)
			{
				if (idx + HDR_SIZE + hdr.N * 2 + 2 > len)
				{
					// need more bytes
					break;
				}

				RawData *fan = GetData0xCE_2(hdr, buf + idx, parser->flags, parser->with_chk);
				if (fan)
				{
					fans[*nfan] = fan;
					*nfan += 1;
				}

				idx += HDR_SIZE + hdr.N * 2 + 2;
			}
			else
			{
				if (idx + HDR_SIZE + hdr.N * 3 + 2 > len)
				{
					// need more bytes
					break;
				}
				RawData *fan = GetData0xCE_3(hdr, buf + idx, parser->flags, parser->with_chk);
				if (fan)
				{
					fans[*nfan] = fan;
					*nfan += 1;
				}
				idx += HDR_SIZE + hdr.N * 3 + 2;
			}
		}
		else if (type == 0x99)
		{
			if (idx + hdr.N * 3 + HDR99_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}

			RawDataHdr99 hdr99;
			memcpy(&hdr99, buf + idx, HDR99_SIZE);

			RawData *fan = GetData0x99(hdr99, buf + idx, parser->with_chk);
			if (fan)
			{
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N * 3 + HDR99_SIZE + 2;
		}

		else if (type == 0xCF)
		{
			if (idx + hdr.N * 3 + HDR2_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
			RawDataHdr2 hdr2;
			memcpy(&hdr2, buf + idx, HDR2_SIZE);

			RawData *fan = GetData0xCF(hdr2, buf + idx, parser->with_chk);
			if (fan)
			{
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N * 3 + HDR2_SIZE + 2;
		}
		else if (type == 0xC7)
		{
			if (idx + hdr.N * 5 + HDR7_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
			RawDataHdr7 hdr7;
			memcpy(&hdr7, buf + idx, HDR7_SIZE);

			RawData *fan = GetData0xC7(parser, hdr7, buf + idx);
			if (fan)
			{
				// printf("set [%d] %d\n", *nfan, fan->angle);
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N * 5 + HDR7_SIZE + 2;
		}
		else if (type == 0xDF)
		{
			if (idx + hdr.N * 3 + HDR3_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
			RawDataHdr3 hdr3;
			memcpy(&hdr3, buf + idx, HDR3_SIZE);

			RawData *fan = GetData0xDF(hdr3, buf + idx, parser->with_chk);
			if (fan)
			{
				// printf("set [%d] %d\n", *nfan, fan->angle);
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N * 3 + HDR3_SIZE + 2;
		}
		else
		{
			// should not
		}
	}
	return idx;
}

HParser ParserOpen(int raw_bytes, uint32_t device_ability, uint32_t init_states,
				   int init_rpm, double resample_res, bool with_chksum, uint32_t dev_id,int error_circle,double error_scale)
{
	Parser *parser = new Parser;

	parser->rest_len = 0;
	parser->raw_mode = raw_bytes;
	parser->init_rpm = init_rpm;
	parser->with_chk = with_chksum;
	parser->device_ability = device_ability;
	parser->init_states = init_states;
	parser->flags = init_states;
	parser->resample_res = resample_res;
	parser->fan_segs = NULL;
	parser->dev_id = dev_id;
	parser->error_circle = error_circle;
	parser->error_scale = error_scale;
	return parser;
}

int ParserClose(HParser hP)
{
	Parser *parser = (Parser *)hP;
	delete parser;
	return 0;
}

int ParserRunStream(HParser hP, int len, unsigned char *bytes, RawData *fans[])
{
	Parser *parser = (Parser *)hP;

	int nfan = MAX_FANS;

	unsigned char *buf = new unsigned char[len + parser->rest_len];
	memset(buf, 0, sizeof(len + parser->rest_len));
	if (!buf)
	{
		printf("out of memory\n");
		return -1;
	}

	if (parser->rest_len > 0)
	{
		memcpy(buf, parser->rest_buf, parser->rest_len);
	}
	memcpy(buf + parser->rest_len, bytes, len);
	len += parser->rest_len;

	int used = ParseStream(parser, len, buf, &nfan, fans);

#if 0
	for (int i=0; i<nfan; i++) 
	{
		fans[i]->ros_angle = LidarAng2ROS(fans[i]->angle + fans[i]->span);
	}
#endif

	parser->rest_len = len - used;
	if (parser->rest_len > 0)
	{
		memcpy(parser->rest_buf, buf + used, parser->rest_len);
	}

	delete[] buf;

	return nfan;
}
int alarmProc(unsigned char *buf, int len)
{
	//报�?�信�?打印
	if (memcmp(buf, "LMSG", 4) == 0)
	{
		if (len >= (int)sizeof(LidarAlarm))
		{
			LidarAlarm *msg = (LidarAlarm *)buf;
			if (msg->hdr.type >= 0x100)
			{
				//说明有LMSG_ALARM报�?�信�?
				if (getbit(msg->hdr.data, 12) == 1)
				{
					printf("ALARM LEVEL:OBSERVE  MSG TYPE:%d ZONE ACTIVE:%x\n", msg->hdr.type, msg->zone_actived);
				}
				if (getbit(msg->hdr.data, 13) == 1)
				{
					printf("ALARM LEVEL:WARM  MSG TYPE:%d ZONE ACTIVE:%x\n", msg->hdr.type, msg->zone_actived);
				}
				if (getbit(msg->hdr.data, 14) == 1)
				{
					printf("ALARM LEVEL:ALARM  MSG TYPE:%d ZONE ACTIVE:%x\n", msg->hdr.type, msg->zone_actived);
				}
				if (getbit(msg->hdr.data, 15) == 1)
				{
					printf("ALARM COVER   MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 16) == 1)
				{
					printf("ALARM NO DATA   MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 17) == 1)
				{
					// printf("ALARM ZONE NO ACTIVE  MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 18) == 1)
				{
					printf("ALARM SYSTEM ERROR  MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 19) == 1)
				{
					printf("ALARM RUN EXCEPTION  MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 20) == 1)
				{
					printf("ALARM NETWORK ERROR  MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 21) == 1)
				{
					printf("ALARM UPDATING  MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 22) == 1)
				{
					printf("ALARM ZERO POS ERROR  MSG TYPE:%d\n", msg->hdr.type);
				}
				//说明有LMSG_ERROR报错信息
				if (msg->hdr.type % 2 == 1)
				{

					if (getbit(msg->hdr.data, 0) == 1)
					{
						printf("LIDAR LOW POWER  MSG TYPE:%d\n", msg->hdr.type);
					}
					if (getbit(msg->hdr.data, 1) == 1)
					{
						printf("LIDAR  MOTOR STALL  MSG TYPE:%d\n", msg->hdr.type);
					}
					if (getbit(msg->hdr.data, 2) == 1)
					{
						printf("LIDAR RANGING MODULE TEMPERATURE HIGH  MSG TYPE:%d\n", msg->hdr.type);
					}
					if (getbit(msg->hdr.data, 3) == 1)
					{
						printf("LIDAR NETWORK ERROR  MSG TYPE:%d\n", msg->hdr.type);
					}
					if (getbit(msg->hdr.data, 4) == 1)
					{
						printf("LIDAR RANGER MODULE NO OUTPUT  MSG TYPE:%d\n", msg->hdr.type);
					}
				}
			}
		}

		return 1;
	}
	return 0;
}
int ParserRun(LidarNode hP, int len, unsigned char *buf, RawData *fans[])
{
	Parser *parser = (Parser *)hP.hParser;

	uint8_t type = buf[0];
	//报�?�信�?打印
	if (alarmProc(buf, len))
		return 0;

	if (buf[1] != 0xfa)
	{
		// printf("skip packet %x %x len\n", buf[0], buf[1]);
	}
	else if (buf[0] == 0x88)
	{
		PacketUart hdr;
		memcpy(&hdr, buf, sizeof(PacketUart));
		if ((unsigned int)len >= hdr.len + sizeof(PacketUart))
		{
			if (parser->dev_id != (u_int32_t)ANYONE && hdr.dev_id != parser->dev_id)
			{
				// not my data
				return 0;
			}
			return ParserRunStream(hP.hParser, hdr.len, buf + sizeof(PacketUart), fans);
		}
	}
	else if (type == 0xCE)
	{
		RawDataHdr hdr;
		memcpy(&hdr, buf, HDR_SIZE);

		if (HDR_SIZE + hdr.N * 2 + 2 == len)
		{
			RawData *fan = GetData0xCE_2(hdr, buf, parser->flags, parser->with_chk);
			if (fan)
			{
				fans[0] = fan;
				return 1;
			}
		}
		else if (HDR_SIZE + hdr.N * 3 + 2 == len)
		{
			RawData *fan = GetData0xCE_3(hdr, buf, parser->flags, parser->with_chk);
			if (fan)
			{
				fans[0] = fan;
				return 1;
			}
		}
		else
			printf("CE len %d N %d\n", len, hdr.N);
	}
	else if (type == 0xCF)
	{
		RawDataHdr2 hdr;
		memcpy(&hdr, buf, HDR2_SIZE);

		if (hdr.N * 3 + HDR2_SIZE + 2 > len)
		{
			// need more bytes
			printf("CF len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData *fan = GetData0xCF(hdr, buf, parser->with_chk);
		if (fan)
		{
			// printf("CF %d + %d %d\n", fan->angle, fan->span, fan->N);
			fans[0] = fan;
			return 1;
		}
	}
	else if (type == 0xDF)
	{
		RawDataHdr3 hdr;
		memcpy(&hdr, buf, HDR3_SIZE);

		if (hdr.N * 3 + HDR3_SIZE + 2 > len)
		{
			// need more bytes
			printf("DF len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData *fan = GetData0xDF(hdr, buf, parser->with_chk);
		if (fan)
		{
			// printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
	}
	else if (type == 0xC7)
	{
		RawDataHdr7 hdr;
		memcpy(&hdr, buf, HDR7_SIZE);

		if (hdr.N * 5 + HDR7_SIZE + 2 > len)
		{
			// need more bytes
			// printf("C7 len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData *fan = GetData0xC7(parser, hdr, buf);
		if (fan)
		{
			// printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
		return 0;
	}
	else if (type == 0x99)
	{
		RawDataHdr99 hdr;
		memcpy(&hdr, buf, HDR99_SIZE);

		if (hdr.N * 3 + HDR99_SIZE + 2 > len)
		{
			// need more bytes
			// printf("99 len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData *fan = GetData0x99(hdr, buf, parser->with_chk);
		if (fan)
		{
			// printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
		return 0;
	}

	if (buf[0] == 0x4c && buf[1] == 0x48)
	{
		//
		return 0;
	}
	printf("skip packet %08x len %d\n", *(uint32_t *)buf, len);
	return 0;
}

int strip(const char *s, char *buf)
{
	int len = 0;
	for (int i = 0; s[i] != 0; i++)
	{
		if (s[i] >= 'a' && s[i] <= 'z')
			buf[len++] = s[i];
		else if (s[i] >= 'A' && s[i] <= 'Z')
			buf[len++] = s[i];
		else if (s[i] >= '0' && s[i] <= '9')
			buf[len++] = s[i];
		else if (len > 0)
			break;
	}
	buf[len] = 0;
	return len;
}

char g_uuid[32] = "";

bool ParserScript(HParser hP, Script script, S_PACK s_pack, const char *type, void *hnd)
{
	
	Parser *parser = (Parser *)hP;
	unsigned int index = 5;
	char buf[32];
	char result[3]={0};
	result[2]='\0';
	if (parser->device_ability & DF_WITH_UUID)
	{
		for (unsigned int i = 0; i < index; i++)
		{
			if (script(hnd, 6, "LUUIDH", 11, "PRODUCT SN:", 16, buf))
			{
				strip(buf, g_uuid);
				printf("get product SN : \'%s\'\n", g_uuid);
				break;
			}
			else if (script(hnd, 6, "LUUIDH", 10, "VENDOR ID:", 16, buf))
			{
				strip(buf, g_uuid);
				printf("get product SN : \'%s\'\n", g_uuid);
				break;
			}
		}
	}

	// setup output data format
	if (parser->device_ability & DF_UNIT_IS_MM)
	{
		for (unsigned int i = 0; i < index; i++)
		{
			const char *cmd = (parser->init_states & DF_UNIT_IS_MM) ? "LMDMMH" : "LMDCMH";
			if (script(hnd, 6, cmd, 10, "SET LiDAR ", 12, buf))
			{
				printf("set LiDAR unit %s \n", buf);
				break;
			}
		}
	}

	// enable/disable output intensity
	if (parser->device_ability & DF_WITH_INTENSITY)
	{
		for (unsigned int i = 0; i < index; i++)
		{
			const char *cmd = (parser->init_states & DF_WITH_INTENSITY) ? "LOCONH" : "LNCONH";
			if (script(hnd, 6, cmd, 6, "LiDAR ", 12, buf))
			{
				printf("set LiDAR confidence %s \n", buf);
				break;
			}
		}
	}

	// enable/disable shadow filter
	if (parser->device_ability & DF_DESHADOWED)
	{
		if (strcmp(type, "udp") == 0)
		{
			for (unsigned int i = 0; i < index; i++)
			{
				const char *cmd = (parser->init_states & DF_DESHADOWED) ? "LSDSW:1H" : "LSDSW:0H";
				if (s_pack(hnd, strlen(cmd), cmd,result))
				{
					printf("set LiDAR shadow filter %s %s\n", cmd,result);
					break;
				}
			}
		}
		else
		{
			for (unsigned int i = 0; i < index; i++)
			{
				const char *cmd = (parser->init_states & DF_DESHADOWED) ? "LFFF1H" : "LFFF0H";
				if (script(hnd, 6, cmd, 6, "LiDAR ", 12, buf))
				{
					printf("set LiDAR shadow filter %s \n", buf);
					break;
				}
			}
		}
	}

	// enable/disable smooth
	if (parser->device_ability & DF_SMOOTHED)
	{
		if (strcmp(type, "udp") == 0)
		{
			for (unsigned int i = 0; i < index; i++)
			{
				const char *cmd = (parser->init_states & DF_SMOOTHED) ? "LSSMT:1H" : "LSSMT:0H";
				if (s_pack(hnd, strlen(cmd), cmd,result))
				{
					printf("set LiDAR smooth  %s %s\n", cmd,result);
					break;
				}
			}
		}
		else
		{
			for (unsigned int i = 0; i < index; i++)
			{
				const char *cmd = (parser->init_states & DF_SMOOTHED) ? "LSSS1H" : "LSSS0H";
				if (script(hnd, 6, cmd, 6, "LiDAR ", 12, buf))
				{
					printf("set LiDAR smooth to %s\n", buf);
					break;
				}
			}
		}
	}

	// setup rpm
	if (parser->device_ability & DF_WITH_RPM)
	{
		if (parser->init_rpm > 300 && parser->init_rpm < 3000)
		{
			for (unsigned int i = 0; i < index; i++)
			{
				char cmd[32];
				sprintf(cmd, "LSRPM:%dH", parser->init_rpm);
				if (strcmp(type, "udp") == 0)
				{
					if (s_pack(hnd, strlen(cmd), cmd,result))
					{
						printf("set RPM to %d %s\n", parser->init_rpm,result);
						break;
					}
				}
				else
				{
					if (script(hnd, strlen(cmd), cmd, 3, "RPM", 12, buf))
					{
						printf("set RPM to %d %s\n", parser->init_rpm,buf);
						break;
					}
				}
			}
		}
	}

	if (parser->device_ability & DF_WITH_RESAMPLE)
	{
		for (unsigned int i = 0; i < index; i++)
		{
			char cmd[32] = "";
			if (parser->init_states & DF_WITH_RESAMPLE)
			{
				if (parser->resample_res == 0 || parser->resample_res == 1)
				{
					sprintf(cmd, "LSRES:%dH", int(parser->resample_res));
				}
				else
				{
					sprintf(cmd, "LSRES:%dH", int(parser->resample_res * 1000));
				}
				char pattern[] = "set resolution ";
				if (strcmp(type, "udp") == 0)
				{
					if (script(hnd, strlen(cmd), cmd, 2, "OK", 0, NULL))
					{
						printf("set resolution %s OK\n", cmd);
						break;
					}	
				}	
				else
				{
					if (script(hnd, strlen(cmd), cmd, strlen(pattern), pattern, 12, buf))
					{
						printf("set resolution %s\n", buf);
						break;
					}
				}
			}
		}
	}
	// enable/disable alaram message uploading
	if (parser->device_ability & EF_ENABLE_ALARM_MSG)
	{
		char cmd[32];
		sprintf(cmd, "LSPST:%dH", (parser->init_states & EF_ENABLE_ALARM_MSG) ? 3 : 1);
		for (unsigned int i = 0; i < index; i++)
		{
			if (s_pack(hnd, strlen(cmd), cmd,result))
			{
				printf("set alarm_msg %s\n",result);
				break;
			}
		}
	}

	return true;
}
void saveLog(bool isSaveLog, const char *logPath, int type, const char *ip,const int port,const unsigned char *buf, unsigned int len)
{
	if (isSaveLog)
	{
		FILE *fp = fopen(logPath, "aw");
		if (fp)
		{
			if (type == 0)
				fprintf(fp, "%s %d SEND MSG:\t",ip,port);
			if (type == 1)
				fprintf(fp, "%s %d REV MSG:\t",ip,port);

			for (unsigned int i = 0; i < len; i++)
			{
				fprintf(fp, "%02x\t", buf[i]);
			}
			fprintf(fp, "\n");
			fclose(fp);
		}
	}
}

int mkpathAll(std::string s, mode_t mode = 0755)
{
	size_t pre = 0, pos;
	std::string dir;
	int mdret;

	if (s[s.size() - 1] != '/')
	{
		// force trailing / so we can handle everything in loop
		s += '/';
	}

	while ((pos = s.find_first_of('/', pre)) != std::string::npos)
	{
		dir = s.substr(0, pos++);
		pre = pos;
		if (dir.size() == 0)
			continue; // if leading / first time is 0 length
		if ((mdret = ::mkdir(dir.c_str(), mode)) && errno != EEXIST)
		{
			return mdret;
		}
	}
	return mdret;
}
