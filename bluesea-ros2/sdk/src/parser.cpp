#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <stdlib.h>
#include "../include/parser.h"
char g_uuid[32] = "";
char g_model[16] = "";
int g_timemode = 0;

void timestampMode(int type)
{
	g_timemode = type;
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

	if (buf[0] == 0xCE || buf[0] == 0xCF || buf[0] == 0xDF || buf[0] == 0xC7 || buf[0] == 0xAA)
		return buf[0];

	return 0;
}

static RawData *GetData0xCE_2(const RawDataHdr &hdr, unsigned char *buf, uint32_t flags, bool with_chk)
{
	RawData *dat = new RawData;
	if (!dat)
	{
		ERROR((int)MEMORY_SPACE_ERR, Error::GetErrorString(MEMORY_SPACE_ERR).c_str());
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	memcpy(dat, &hdr, HDR_SIZE);
	dat->span = 360;

	int is_mm = getbit(flags, DF_UNIT_IS_MM);
	int with_conf = getbit(flags, DF_WITH_INTENSITY);

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
			// DEBUG("%d %d",is_mm,dat->points[i].distance);
			//  if(val&0x7)
			//  	DEBUG("%d",val&0x7);

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
		// ERROR((int)CHECKSUM_ERR, Error::GetErrorString(CHECKSUM_ERR).c_str());
		printf("chksum ce 2 error\n");
		return NULL;
	}
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
		printf("chksum ce 3 error\n");
		return NULL;
	}

	// memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	// printf("get3 %d(%d)\n", hdr.angle, hdr.N);

	SetTimeStamp(dat);
	return dat;
}

static FanSegment_C7 *GetFanSegment(const RawDataHdr7 &hdr, uint8_t *pdat, bool /*with_chk*/)
{
	FanSegment_C7 *fan_seg = new FanSegment_C7;
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
static FanSegment_AA *GetFanSegment(const RawDataHdrAA &hdr, uint8_t *pdat, bool /*with_chk*/)
{
	FanSegment_AA *fan_seg = new FanSegment_AA;
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
		for (int i = 1; i < HDRAA_SIZE / 2; i++)
			sum += pchk[i];
	}

	uint8_t *pDist = pdat + HDRAA_SIZE;
	uint8_t *pAngle = pdat + HDRAA_SIZE + 2 * hdr.N;
	uint8_t *energy = pdat + HDRAA_SIZE + 4 * hdr.N;

	for (int i = 0; i < hdr.N; i++, pDist += 2, pAngle += 2)
	{
		fan_seg->dist[i] = ((uint16_t)(pDist[1]) << 8) | pDist[0];
		fan_seg->angle[i] = ((uint16_t)(pAngle[1]) << 8) | pAngle[0];
		fan_seg->energy[i] = energy[i];

		sum += fan_seg->dist[i];
		sum += fan_seg->angle[i];
		sum += energy[i];
	}

	uint8_t *pchk = pdat + HDRAA_SIZE + 5 * hdr.N;
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

static RawData *PackFanData(FanSegment_C7 *seg)
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
	dat->counterclockwise = (seg->hdr.flags & DF_MOTOR_REVERSE) ? 1 : 0;

	if (!g_timemode)
		SetTimeStamp(dat);
	else
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
static RawData *PackFanData(FanSegment_AA *seg)
{
	RawData *dat = new RawData;
	if (!dat)
	{
		printf("out of memory\n");
		return NULL;
	}
	memset(dat, 0, sizeof(RawData));

	dat->code = 0xfaaa;
	dat->N = seg->hdr.whole_fan;
	dat->angle = seg->hdr.beg_ang / 100;					 // 0.1 degree
	dat->span = (seg->hdr.end_ang - seg->hdr.beg_ang) / 100; // 0.1 degree
	dat->fbase = 0;
	dat->first = 0;
	dat->last = 0;
	dat->fend = 0;
	dat->counterclockwise = (seg->hdr.flags & DF_MOTOR_REVERSE) ? 1 : 0;

	if (!g_timemode)
		SetTimeStamp(dat);
	else
	{
		dat->ts[0] = seg->hdr.second;
		dat->ts[1] = seg->hdr.nano_sec / 1000;
	}

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
static int GetFanPointCount(FanSegment_C7 *seg)
{
	int n = 0;

	while (seg)
	{
		n += seg->hdr.N;
		seg = seg->next;
	}

	return n;
}
static int GetFanPointCount(FanSegment_AA *seg)
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

	FanSegment_C7 *fan_seg = GetFanSegment(hdr, pdat, parser->with_chk);
	if (!fan_seg)
	{
		return NULL;
	}

	// printf("fan %d %d\n", fan_seg->hdr.beg_ang, fan_seg->hdr.ofset);

	if (parser->fan_segs_c7 != NULL)
	{
		FanSegment_C7 *seg = parser->fan_segs_c7;

		if (seg->hdr.timestamp != fan_seg->hdr.timestamp)
		{
			printf("drop old fan segments\n");
			while (seg)
			{
				parser->fan_segs_c7 = seg->next;
				delete seg;
				seg = parser->fan_segs_c7;
			}
			parser->fan_segs_c7 = fan_seg;
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

	if (parser->fan_segs_c7 == NULL && fan_seg != NULL)
	{
		parser->fan_segs_c7 = fan_seg;
	}

	// if (parser->fan_segs == NULL) { return NULL; }

	unsigned int N = GetFanPointCount(parser->fan_segs_c7);

	if (N >= parser->fan_segs_c7->hdr.whole_fan)
	{
		RawData *dat = NULL;

		if (N == parser->fan_segs_c7->hdr.whole_fan)
		{
			if (N > sizeof(dat->points) / sizeof(dat->points[0]))
			{
				printf("too many %d points in 1 fan\n", N);
			}
			else
			{
				dat = PackFanData(parser->fan_segs_c7);
			}
		}

		// remove segments
		FanSegment_C7 *seg = parser->fan_segs_c7;
		while (seg)
		{
			parser->fan_segs_c7 = seg->next;
			delete seg;
			seg = parser->fan_segs_c7;
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

static RawData *GetData0xAA(Parser *parser, const RawDataHdrAA &hdr, uint8_t *pdat)
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
	FanSegment_AA *fan_seg = GetFanSegment(hdr, pdat, parser->with_chk);
	if (!fan_seg)
	{
		return NULL;
	}

	// printf("fan %d %d\n", fan_seg->hdr.beg_ang, fan_seg->hdr.ofset);

	if (parser->fan_segs_aa != NULL)
	{
		FanSegment_AA *seg = parser->fan_segs_aa;

		if ((seg->hdr.second != fan_seg->hdr.second) || (seg->hdr.nano_sec != fan_seg->hdr.nano_sec))
		{
			printf("drop old fan segments\n");
			while (seg)
			{
				parser->fan_segs_aa = seg->next;
				delete seg;
				seg = parser->fan_segs_aa;
			}
			parser->fan_segs_aa = fan_seg;
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

	if (parser->fan_segs_aa == NULL && fan_seg != NULL)
	{
		parser->fan_segs_aa = fan_seg;
	}

	// if (parser->fan_segs == NULL) { return NULL; }

	unsigned int N = GetFanPointCount(parser->fan_segs_aa);

	if (N >= parser->fan_segs_aa->hdr.whole_fan)
	{
		RawData *dat = NULL;

		if (N == parser->fan_segs_aa->hdr.whole_fan)
		{
			if (N > sizeof(dat->points) / sizeof(dat->points[0]))
			{
				printf("too many %d points in 1 fan\n", N);
			}
			else
			{
				dat = PackFanData(parser->fan_segs_aa);
			}
		}

		// remove segments
		FanSegment_AA *seg = parser->fan_segs_aa;
		while (seg)
		{
			parser->fan_segs_aa = seg->next;
			delete seg;
			seg = parser->fan_segs_aa;
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
	if (!g_timemode)
		SetTimeStamp(dat);
	else
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
	dat->counterclockwise = -1;
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
	dat->counterclockwise = -1;
	return dat;
}

static int MsgProc(Parser *parser, int len, unsigned char *buf)
{
	if (len >= 8 && buf[0] == 'S' && buf[1] == 'T' && buf[6] == 'E' && buf[7] == 'D')
	{
		parser->flags = update_flags(buf + 2);
		return 1;
	}
	else if (alarmProc(buf, len))
	{
		return 2;
	}
	else if (len >= 4 && buf[0] == 0xce && buf[1] == 0xce)
	{
		if (buf[2] == 0x0 && buf[3] == 0x01)
		{
			ERROR(ALARM_LOWPOWER, Error::GetErrorString(ALARM_LOWPOWER).c_str());
		}
		else if (buf[2] == 0x0 && buf[3] == 0x02)
		{
			ERROR(ALARM_ZERO_ERR, Error::GetErrorString(ALARM_ZERO_ERR).c_str());
		}
		else if (buf[2] == 0x0 && buf[3] == 0x03)
		{
			ERROR(ALARM_CODEDISK_ERR, Error::GetErrorString(ALARM_CODEDISK_ERR).c_str());
		}
		else if (buf[2] == 0x0 && buf[3] == 0x04)
		{
			ERROR(ALARM_RESET_ERR, Error::GetErrorString(ALARM_RESET_ERR).c_str());
		}
		else if (buf[2] == 0x0 && buf[3] == 0x05)
		{
			ERROR(ALARM_MOUDLE_ERR, Error::GetErrorString(ALARM_MOUDLE_ERR).c_str());
		}
		return 3;
	}
	else
	{
		printf("unknown %d bytes : %02x ", len, buf[0]);
		for (int i = 1; i < len; i++)
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
	unsigned char unknown[1024];
	while (idx < len - 128 && *nfan < max_fan)
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
			int ret = MsgProc(parser, unk, unknown);
			if (ret == 1)
			{
				int8_t flag = parser->flags >> 24;
				if (strcmp(g_model, "LDS-50C-R") == 0 || strcmp(g_model, "LDS-E200-R") == 0 || strcmp(g_model, "LDS-E200-A") == 0 || strcmp(g_model, "LDS-E200-A4") == 0
				|| strcmp(g_model, "LDS-E210-R") == 0|| strcmp(g_model, "LDS-E210-A") == 0)
				{
					if (flag & 0x1)
					{
						ERROR((int)ALARM_BOTTOMPLATE_LOW_VOLTAGE, Error::GetErrorString(ALARM_BOTTOMPLATE_LOW_VOLTAGE).c_str());
					}
					if (flag & 0x2)
					{
						ERROR((int)ALARM_BOTTOMPLATE_HIGH_VOLTAGE, Error::GetErrorString(ALARM_BOTTOMPLATE_HIGH_VOLTAGE).c_str());
					}
					if (flag & 0x4)
					{
						ERROR((int)ALARM_TEMPERATURE_ERR, Error::GetErrorString(ALARM_TEMPERATURE_ERR).c_str());
					}
					if (flag & 0x8)
					{
						ERROR((int)ALARM_MOTERHEAD_LOW_VOLTAGE, Error::GetErrorString(ALARM_MOTERHEAD_LOW_VOLTAGE).c_str());
					}
					if (flag & 0x10)
					{
						ERROR((int)ALARM_MOTERHEAD_HIGH_VOLTAGE, Error::GetErrorString(ALARM_MOTERHEAD_HIGH_VOLTAGE).c_str());
					}
				}
			}
			else if (ret == -1)
			{
				parser->rest_len = 0;
				memset(parser->rest_buf, 0, sizeof(parser->rest_buf));
			}
			unk = 0;
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf + idx, HDR_SIZE);

		if (hdr.N > MAX_POINTS)
		{
			ERROR((int)POINT_NUM_LARGE, Error::GetErrorString(POINT_NUM_LARGE).c_str());
			idx += 2;
			continue;
		}

		if (type == 0xCE)
		{
			int raw_mode = 3;
			if (idx + HDR_SIZE + hdr.N * 3 + 2 > len)
				break;

			if (parser->raw_mode == 3)
			{
				RawData *fan = GetData0xCE_3(hdr, buf + idx, parser->flags, parser->with_chk);
				if (fan)
				{
					fans[*nfan] = fan;
					*nfan += 1;
					idx += HDR_SIZE + hdr.N * 3 + 2;
					fan->counterclockwise = -1;
				}
				else
				{
					if (!parser->isrun)
					{
						fan = GetData0xCE_2(hdr, buf + idx, parser->flags, parser->with_chk);
						if (fan)
						{
							fans[*nfan] = fan;
							*nfan += 1;
							raw_mode = 2;
							idx += HDR_SIZE + hdr.N * 2 + 2;
							fan->counterclockwise = 1;
						}
						else
							idx += HDR_SIZE + hdr.N * 3 + 2;
					}
					else
						idx += HDR_SIZE + hdr.N * 3 + 2;
				}
			}
			else
			{
				RawData *fan = GetData0xCE_2(hdr, buf + idx, parser->flags, parser->with_chk);
				if (fan)
				{
					fans[*nfan] = fan;
					*nfan += 1;
					idx += HDR_SIZE + hdr.N * 2 + 2;
					raw_mode = 2;
					fan->counterclockwise = 1;
				}
				else
				{
					if (!parser->isrun)
					{
						fan = GetData0xCE_3(hdr, buf + idx, parser->flags, parser->with_chk);
						if (fan)
						{
							fans[*nfan] = fan;
							*nfan += 1;
							idx += HDR_SIZE + hdr.N * 3 + 2;
							fan->counterclockwise = -1;
						}
						else
							idx += HDR_SIZE + hdr.N * 2 + 2;
					}
					else
						idx += HDR_SIZE + hdr.N * 2 + 2;
				}
			}
			if (!parser->isrun)
				parser->raw_mode = raw_mode;
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
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N * 5 + HDR7_SIZE + 2;
		}
		else if (type == 0xAA)
		{
			if (idx + hdr.N * 5 + HDRAA_SIZE + 2 > len)
			{
				break;
			}
			RawDataHdrAA hdrAA;
			memcpy(&hdrAA, buf + idx, HDRAA_SIZE);

			RawData *fan = GetData0xAA(parser, hdrAA, buf + idx);
			if (fan)
			{
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N * 5 + HDRAA_SIZE + 2;
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
	return idx - unk;
}

Parser *ParserOpen(int raw_bytes, bool with_chksum, int dev_id,
				   int error_circle, double error_scale, bool from_zero, int time_mode,
				   CommandList cmd, char *ip, int port)
{
	Parser *parser = new Parser;

	parser->rest_len = 0;
	parser->raw_mode = raw_bytes;
	parser->with_chk = with_chksum;
	parser->fan_segs_c7 = NULL;
	parser->fan_segs_aa = NULL;
	parser->dev_id = dev_id;
	parser->error_circle = error_circle;
	parser->error_scale = error_scale;
	parser->cmd = cmd;
	parser->is_from_zero = from_zero;
	strcpy(parser->ip, ip);
	parser->port = port;
	g_timemode = time_mode;
	parser->isrun = false;
	return parser;
}

// int ParserClose(HParser hP)
// {
// 	Parser *parser = (Parser *)hP;
// 	delete parser;
// 	return 0;
// }

int ParserRunStream(Parser *hP, int len, unsigned char *bytes, RawData *fans[])
{
	Parser *parser = (Parser *)hP;

	int nfan = MAX_FANS;

	unsigned char *buf = new unsigned char[len + parser->rest_len];
	memset(buf, 0, sizeof(len + parser->rest_len));
	if (!buf)
	{
		// printf("out of memory\n");
		ERROR((int)MEMORY_SPACE_ERR, Error::GetErrorString(MEMORY_SPACE_ERR).c_str());
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
	if (memcmp(buf, "LMSG", 4) == 0)
	{
		if (len >= (int)sizeof(LidarAlarm))
		{
			LidarAlarm *msg = (LidarAlarm *)buf;
			if (msg->hdr.type >= 0x100)
			{
				if (getbit(msg->hdr.data, 12) == 1)
				{
					DEBUG("ALARM LEVEL:OBSERVE  MSG TYPE:%d ZONE ACTIVE:%x", msg->hdr.type, msg->zone_actived);
				}
				if (getbit(msg->hdr.data, 13) == 1)
				{
					DEBUG("ALARM LEVEL:WARM  MSG TYPE:%d ZONE ACTIVE:%x", msg->hdr.type, msg->zone_actived);
				}
				if (getbit(msg->hdr.data, 14) == 1)
				{
					DEBUG("ALARM LEVEL:ALARM  MSG TYPE:%d ZONE ACTIVE:%x", msg->hdr.type, msg->zone_actived);
				}
				if (getbit(msg->hdr.data, 15) == 1)
				{
					DEBUG("ALARM COVER   MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 16) == 1)
				{
					DEBUG("ALARM NO DATA   MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 17) == 1)
				{
					// printf("ALARM ZONE NO ACTIVE  MSG TYPE:%d\n", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 18) == 1)
				{
					DEBUG("ALARM SYSTEM ERROR  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 19) == 1)
				{
					DEBUG("ALARM RUN EXCEPTION  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 20) == 1)
				{
					DEBUG("ALARM NETWORK ERROR  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 21) == 1)
				{
					DEBUG("ALARM UPDATING  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 22) == 1)
				{
					DEBUG("ALARM ZERO POS ERROR  MSG TYPE:%d", msg->hdr.type);
				}
			}
			// 说明有LMSG_ERROR报错信息
			if (msg->hdr.type % 2 == 1)
			{

				if (getbit(msg->hdr.data, 0) == 1)
				{
					DEBUG("LIDAR LOW POWER  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 1) == 1)
				{
					DEBUG("LIDAR  MOTOR STALL  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 2) == 1)
				{
					DEBUG("LIDAR RANGING MODULE TEMPERATURE HIGH  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 3) == 1)
				{
					DEBUG("LIDAR NETWORK ERROR  MSG TYPE:%d", msg->hdr.type);
				}
				if (getbit(msg->hdr.data, 4) == 1)
				{
					DEBUG("LIDAR RANGER MODULE NO OUTPUT  MSG TYPE:%d", msg->hdr.type);
				}
			}
		}

		return 1;
	}
	if (memcmp(buf, "AM", 2) == 0)
	{
		if (len >= (int)sizeof(PROCOTOL_HOST_ALARM_ST))
		{
			PROCOTOL_HOST_ALARM_ST *msg = (PROCOTOL_HOST_ALARM_ST *)buf;
			// crc校验   有可能是脏数据
			uint32_t crc = msg->crc;
			uint32_t crc2 = stm32crc((unsigned int *)(msg), (sizeof(PROCOTOL_HOST_ALARM_ST) - 4) / 4);
			if (crc == crc2)
			{
				// 错误信息
				if (getbit(msg->events, 0) == 1)
				{
					// 底板低压
					DEBUG("mainboard low voltage");
				}
				if (getbit(msg->events, 1) == 1)
				{
					// 底板高压;
					DEBUG("mainboard's voltage too high");
				}
				if (getbit(msg->events, 2) == 1)
				{
					// 机头高温;
					DEBUG("The distance measuring module is too hot");
				}
				if (getbit(msg->events, 3) == 1)
				{
					// 机头低压;
					DEBUG("Ranging Module low voltage");
				}
				if (getbit(msg->events, 4) == 1)
				{
					// 机头高压;
					DEBUG("Ranging Module's voltage too high");
				}
				if (getbit(msg->events, 5) == 1)
				{
					// 机头无数据;
					DEBUG("Ranging Module is Muted");
				}
				if (getbit(msg->events, 6) == 1)
				{
					// 机头数据异常;
					DEBUG("Ranging Module Communication error");
				}
				if (getbit(msg->events, 7) == 1)
				{
					// 红外接收异常;
					DEBUG("IR Communication error");
				}
				if (getbit(msg->events, 8) == 1)
				{
					// 线圈断开;
					DEBUG("Wireless Power Supply Lost");
				}
				if (getbit(msg->events, 9) == 1)
				{
					// 线圈过流;
					DEBUG("Wireless Power Supply  overcurrent");
				}
				if (getbit(msg->events, 10) == 1)
				{
					// 码盘错误;
					DEBUG("encoder exception");
				}
				if (getbit(msg->events, 11) == 1)
				{
					// 电机异常;
					DEBUG("motor exception");
				}
				if (getbit(msg->events, 16) == 1)
				{
					// 观察区;
					DEBUG("Observe");
				}
				if (getbit(msg->events, 17) == 1)
				{
					// 警戒区;
					DEBUG("Warning");
				}
				if (getbit(msg->events, 18) == 1)
				{
					// 报警区;
					DEBUG("Alarm");
				}
				if (getbit(msg->events, 19) == 1)
				{
					// 无防区设置;
					DEBUG("NO Zone Set");
				}
				if (getbit(msg->events, 20) == 1)
				{
					// 雷达遮挡;
					DEBUG("Lidar cover");
				}

				return 1;
			}
		}
	}

	return 0;
}
int ParserRun(LidarNode hP, int len, unsigned char *buf, RawData *fans[])
{
	Parser *parser = (Parser *)hP.hParser;

	uint8_t type = buf[0];
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
			DEBUG("CE len %d N %d\n", len, hdr.N);
	}
	else if (type == 0xCF)
	{
		RawDataHdr2 hdr;
		memcpy(&hdr, buf, HDR2_SIZE);

		if (hdr.N * 3 + HDR2_SIZE + 2 > len)
		{
			// need more bytes
			DEBUG("CF len %d N %d\n", len, hdr.N);
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
			DEBUG("DF len %d N %d\n", len, hdr.N);
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
	else if (type == 0xAA)
	{

		RawDataHdrAA hdr;
		memcpy(&hdr, buf, HDRAA_SIZE);

		if (hdr.N * 5 + HDRAA_SIZE + 2 > len)
		{
			// need more bytes
			// printf("C7 len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData *fan = GetData0xAA(parser, hdr, buf);
		if (fan)
		{
			// printf("set [%d] %d\n",fan->N, fan->angle);
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
	if (buf[0] == 0x4f && buf[1] == 0x4f && buf[2] == 0x42 && buf[3] == 0x53)
	{
		return 0;
	}
	DEBUG("skip packet %08x len %d\n", *(uint32_t *)buf, len);
	// printf("qwer:%02X %02X %02X %02X\n", buf[0],buf[1],buf[2],buf[3]);
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
		else if (s[i] == '-')
			buf[len++] = s[i];
		else if (len > 0)
			break;
	}
	buf[len] = 0;
	return len;
}

int mkpathAll(std::string s, mode_t mode = 0755)
{
	size_t pre = 0, pos;
	std::string dir;
	int mdret;

	if (s[s.size() - 1] != '/')
	{
		// forctrailing / so we can handle everything in loop
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
std::string stringfilter(char *str, int num)
{
	int index = 0;
	for (int i = 0; i < num; i++)
	{
		if ((str[i] >= 45 && str[i] <= 58) || (str[i] >= 65 && str[i] <= 90) || (str[i] >= 97 && str[i] <= 122) || str[i] == 32 || str[i] == '_')
		{
			index++;
		}
		else
		{
			std::string arr = str;
			arr = arr.substr(0, index);
			return arr;
		}
	}
	return "";
}

int find(std::vector<RawData> a, int n, int x)
{
	int i;
	int min = abs(a.at(0).angle - x);
	int r = 0;

	for (i = 0; i < n; ++i)
	{
		if (abs(a.at(i).angle - x) < min)
		{
			min = abs(a.at(i).angle - x);
			r = i;
		}
	}

	return a[r].angle;
}
// int autoGetFirstAngle(RawData raw, bool from_zero, std::vector<RawData> &raws, std::string &result)
// {
// 	int angles = 0;
// 	int size = raws.size();
// 	// printf("angle %d  size:%d\n", raw.angle,size);
// 	if (size >= 1)
// 	{
// 		RawData tmp = raws.at(size - 1);
// 		RawData tmp2 = raws.at(0);
// 		if (raw.angle == tmp2.angle)
// 		{
// 			for (int i = 0; i < size; i++)
// 			{
// 				tmp = raws.at(i);
// 				angles += tmp.span;
// 			}
// 			if (angles != 3600)
// 			{
// 				// result="angle sum "+std::to_string(angles);
// 				// printf("angle sum %d size:%d\n", angles,size);
// 				raws.clear();
// 				return -2;
// 			}
// 			else
// 			{
// 				int ret = -1;
// 				if (from_zero)
// 					ret = find(raws, raws.size(), 0);
// 				else
// 					ret = find(raws, raws.size(), 1800);

// 				raws.clear();
// 				return ret;
// 			}
// 		}
// 		if (raw.angle == (tmp.angle + tmp.span) % 3600)
// 		{
// 			// 说明是连续的扇区
// 			raws.push_back(raw);
// 		}
// 	}
// 	else
// 		raws.push_back(raw);
// 	return -1;
// }
// 获取起始点位坐标的位置
int getFirstidx(RawData raw, int angle)
{
	int idx = -1;
	if ((raw.angle <= angle * 10) && ((raw.angle + raw.span) > angle * 10))
	{
		// 下标减一
		idx = 1.0 * (angle * 10 - raw.angle) / raw.span * raw.N;
	}
	return idx;
}

void PublishData(PubHub *pub, int n, RawData **fans)
{
	int skip = 0;
	RawData *drop[MAX_FANS];
	PubHub *hub = (PubHub *)pub;

	pthread_mutex_lock(&hub->mtx);

	// gettimeofday(&begin,NULL);
	// long long beginTime = (long long)begin.tv_sec * 1000000 + (long long)begin.tv_usec;

	if (hub->nfan + n > MAX_FANS)
	{
		int nr = hub->nfan + n - MAX_FANS;

		for (int i = 0; i < nr; i++)
			drop[skip++] = hub->fans[i];

		for (int i = nr; i < hub->nfan; i++)
		{
			hub->fans[i - nr] = hub->fans[i];
		}

		hub->nfan -= nr;
	}

	for (int i = 0; i < n; i++)
	{
		hub->fans[hub->nfan++] = fans[i];
	}

	// gettimeofday(&begin,NULL);
	// long long end = (long long)begin.tv_sec * 1000000 + (long long)begin.tv_usec;
	// DEBUG("time  2 is:%ld\n",end-beginTime);

	pthread_mutex_unlock(&hub->mtx);

	for (int i = 0; i < skip; i++)
	{
		delete drop[i];
	}
}
void SetTimeStamp(RawData *dat)
{
	timeval t;
	gettimeofday(&t, NULL);
	dat->ts[0] = t.tv_sec;
	dat->ts[1] = t.tv_usec * 1000;
}
