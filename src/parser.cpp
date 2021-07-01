#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include "parser.h"

#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16
#define HDR7_SIZE 28 
#define HDR99_SIZE 32 
#define BUF_SIZE 8*1024

struct RawDataHdr
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
};

struct RawDataHdr2
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;
};

struct RawDataHdr3
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;
	unsigned short fbase;
	unsigned short first;
	unsigned short last;
	unsigned short fend;
};

struct RawDataHdr7 {
	uint16_t code;
	uint16_t N;
	uint16_t whole_fan;
	uint16_t ofset;
	uint32_t beg_ang;
	uint32_t end_ang;
	uint32_t flags;
	uint32_t timestamp;
	uint32_t dev_id;
};

struct RawDataHdr99 {
	uint16_t code;
	uint16_t N;
	uint16_t from;
	uint16_t total;
	uint32_t flags;
	uint32_t timestamp;
	uint32_t dev_no;
	uint32_t reserved[3];
};


typedef struct  {
	uint16_t code;
	uint16_t len;
	uint32_t clk;
	uint32_t dev_id;
} PacketUart;

struct FanSegment
{
	RawDataHdr7 hdr; 

	uint16_t dist[MAX_POINTS];
	uint16_t angle[MAX_POINTS];
	uint8_t energy[MAX_POINTS];

	struct FanSegment* next;
};


struct Parser 
{
	bool stream_mode;
	int rest_len;
	unsigned char rest_buf[BUF_SIZE];
	uint32_t flags;
	uint32_t device_ability;
	uint32_t init_states;
	int init_rpm;
	double resample_res;
	bool with_chk;
	int raw_mode;

	uint32_t dev_id;

	FanSegment* fan_segs;
};

#if 0
short LidarAng2ROS(short ang)
{
	//ang = -ang;
	//return ang < -1800 ? ang + 3600 : ang;
	return ang >= 1800 ? ang - 3600 : ang;
}
#endif

static uint32_t update_flags(unsigned char* buf)
{
	uint32_t v;
	memcpy(&v, buf, 4);
	return v;
}

static unsigned char is_data(unsigned char* buf)
{
	if (buf[1] != 0xFA)
		return 0;
		
	if (buf[0] == 0xCE || buf[0] == 0xCF || buf[0] == 0xDF || buf[0] == 0xC7)
		return buf[0];

	return 0;
}

static RawData* GetData0xCE_2(const RawDataHdr& hdr, unsigned char* buf, uint32_t flags, bool with_chk)
{
	RawData* dat = new RawData;
	if (!dat) {
		printf("out of memory\n");
		return NULL;
	}

	memcpy(dat, &hdr, HDR_SIZE);
	dat->span = 360;

	int is_mm = flags & DF_UNIT_IS_MM;
	int with_conf = flags & DF_WITH_INTENSITY;

	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	unsigned char* pdat = buf+HDR_SIZE;
	for (int i=0; i<hdr.N; i++)
	{
		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short val = (v2<<8) | v;

		if (with_conf)
		{
			dat->points[i].confidence = val >> 13;
			dat->points[i].distance = val & 0x1fff;
			if (is_mm == 0) 
				dat->points[i].distance *= 10;
		} else {
			dat->points[i].confidence = is_mm ? val : val*10;
			dat->points[i].confidence = 0;
		}

		//dat->points[i].angle = hdr.angle*10 + 3600 * i / hdr.N;
		dat->points[i].degree = hdr.angle/10.0 + (dat->span * i) / (10.0 * hdr.N);

		sum += val;
	}

	memcpy(&chk, buf+HDR_SIZE+hdr.N*2, 2);

	if (with_chk && chk != sum) 
	{
		delete dat;
		printf("chksum error");
		return NULL;
	}

	//dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);

					
	SetTimeStamp(dat);

	return dat;
}


static RawData* GetData0xCE_3(const RawDataHdr& hdr, unsigned char* buf, uint32_t flags, bool with_chk)
{
	RawData* dat = new RawData;
	if (!dat) {
		printf("out of memory\n");
		return NULL;
	}

	memcpy(dat, &hdr, HDR_SIZE);
	int span = (flags & DF_FAN_90) ? 90 : 360;

	if (hdr.angle == 3420 && (flags & DF_FAN_90))
		span = 180;
	dat->span = span;

	unsigned char* pdat = buf+HDR_SIZE;
	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	for (int i=0; i<hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2<<8) | v;
		sum += vv;
		dat->points[i].distance = vv;
		//dat->points[i].angle = hdr.angle*10 + span * i * 10 / hdr.N;
		dat->points[i].degree = hdr.angle/10.0 + (span * i) / (10.0 * hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk && chk != sum) 
	{
		delete dat;
		printf("chksum ce error");
		return NULL;;
	}

	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);
	
	SetTimeStamp(dat);
	//dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static FanSegment* GetFanSegment(const RawDataHdr7& hdr, uint8_t* pdat, bool with_chk)
{
	FanSegment* fan_seg = new FanSegment;
	if (!fan_seg) {
		printf("out of memory\n");
		return NULL;
	}
	fan_seg->hdr = hdr;
	fan_seg->next = NULL;

	uint16_t sum = 0;
	//if (with_chk)
	{
		uint16_t* pchk = (uint16_t*)pdat;
		for (int i=1; i<HDR7_SIZE/2; i++) 
			sum += pchk[i];
	}


	uint8_t* pDist = pdat + HDR7_SIZE;
       	uint8_t* pAngle = pdat + HDR7_SIZE + 2*hdr.N;
	uint8_t* energy = pdat + HDR7_SIZE + 4*hdr.N;

	for (int i=0; i<hdr.N; i++, pDist+=2, pAngle+=2)
	{
		fan_seg->dist[i] = ((uint16_t)(pDist[1])<<8) | pDist[0];
		fan_seg->angle[i] = ((uint16_t)(pAngle[1])<<8) | pAngle[0];
		fan_seg->energy[i] = energy[i];

		sum += fan_seg->dist[i];
		sum += fan_seg->angle[i];
		sum += energy[i];
	}

	uint8_t* pchk = pdat + HDR7_SIZE + 5*hdr.N;
	uint16_t chksum = ((uint16_t)(pchk[1])<<8) | pchk[0];
	if (chksum != sum) {
		printf("checksum error\n");
		delete fan_seg;
		return NULL;
	}

	return fan_seg;
}

void DecTimestamp(uint32_t ts, uint32_t* ts2)
{	
	timeval tv;
	gettimeofday(&tv, NULL);

	uint32_t sec = tv.tv_sec % 3600;
	if (sec < 5 && ts/1000 > 3595)
	{
		ts2[0] = (tv.tv_sec / 3600 -1) * 3600 + ts/1000;
	}
	else {
		ts2[0] = (tv.tv_sec / 3600) * 3600 + ts/1000;
	}

	ts2[1] = (ts%1000)*1000000;
}

static RawData* PackFanData(FanSegment* seg)
{
	RawData* dat = new RawData;
	if (!dat) {
		printf("out of memory\n");
		return NULL;
	}

	dat->code = 0xfac7;
	dat->N = seg->hdr.whole_fan;
	dat->angle = seg->hdr.beg_ang/100; // 0.1 degree
	dat->span = (seg->hdr.end_ang - seg->hdr.beg_ang)/100; // 0.1 degree
	dat->fbase = 0;
	dat->first = 0;
	dat->last = 0;
	dat->fend = 0;

	DecTimestamp(seg->hdr.timestamp, dat->ts);
	//printf("%d %d.%d\n", dat->angle, dat->ts[0], dat->ts[1]);

	int count = 0;
	while (seg) 
	{
		for (int i=0; i<seg->hdr.N; i++, count++) 
		{
			dat->points[count].confidence = seg->energy[i];
			dat->points[count].distance = seg->dist[i];
			dat->points[count].degree = (seg->angle[i] + seg->hdr.beg_ang) / 1000.0;
		}
	
		seg = seg->next;
	}

	return dat;
}

static int GetFanPointCount(FanSegment* seg) 
{
	int n = 0;

	while (seg) { n += seg->hdr.N;  seg = seg->next; }

	return n;
}


static RawData* GetData0xC7(Parser* parser, const RawDataHdr7& hdr, uint8_t* pdat)
{
	if (parser->dev_id != ANYONE && hdr.dev_id != parser->dev_id) { 
		static time_t last = 0;
		time_t t = time(NULL);
		if (t > last) {
			printf("device id [%d] != my id [%d]\n", hdr.dev_id, parser->dev_id);
			last = t;
		}
		// not my data
		return NULL;
	}
	
	FanSegment* fan_seg = GetFanSegment(hdr, pdat, parser->with_chk);
	if (!fan_seg) {
		return NULL;
	}
		
	//printf("fan %d %d\n", fan_seg->hdr.beg_ang, fan_seg->hdr.ofset);
	
	if (parser->fan_segs != NULL) 
	{
		FanSegment* seg = parser->fan_segs;

	       	if (seg->hdr.timestamp != fan_seg->hdr.timestamp) 
		{
			printf("drop old fan segments\n");
			while (seg) {
				parser->fan_segs = seg->next;
				delete seg;
				seg = parser->fan_segs;
			}
			parser->fan_segs = fan_seg;
		} else {
		       	while (seg) {
				if (seg->hdr.ofset == fan_seg->hdr.ofset) {
					printf("drop duplicated segment\n");
					delete fan_seg;
					fan_seg = NULL;
					break;
				}
				if (seg->next == NULL) {
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

	int N = GetFanPointCount(parser->fan_segs); 

	if (N >= parser->fan_segs->hdr.whole_fan) 
	{
		RawData* dat = NULL;
		
		if (N == parser->fan_segs->hdr.whole_fan) 
		{ 
			if (N > sizeof(dat->points)/sizeof(dat->points[0]) ) 
			{
				printf("too many %d points in 1 fan\n", N);
			}
			else
			{
				dat = PackFanData( parser->fan_segs);
			}
		}

		// remove segments
		FanSegment* seg = parser->fan_segs;
		while (seg) {
			parser->fan_segs = seg->next;
			delete seg;
			seg = parser->fan_segs;
		}

		if (dat) {
			//SetTimeStamp(dat, );
		       	//dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
		}
		
		return dat;
	}
		
	return NULL;
}

static RawData* GetData0x99(const RawDataHdr99& hdr, unsigned char* pdat, bool)
{
	RawData* dat = new RawData;

	if (!dat) {
		printf("out of memory\n");
		return NULL;
	}

	//memset(dat, 0, sizeof(ScanData));
	dat->code = hdr.code;
	dat->N = hdr.N;
	dat->angle = hdr.from * 3600 / hdr.total; // 0.1 degree
	dat->span =  hdr.N * 3600 / hdr.total; // 0.1 degree
	//dat->fbase = ;
	//dat->first;
	//dat->last;
	//dat->fend;
	DecTimestamp(hdr.timestamp, dat->ts);

	pdat += HDR99_SIZE;

	uint8_t* dist = pdat;
	uint8_t* energy = pdat + 2 * hdr.N;

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



static RawData* GetData0xCF(const RawDataHdr2& hdr, unsigned char* pdat, bool with_chk)
{
	RawData* dat = new RawData;
	if (!dat) {
		printf("out of memory\n");
		return NULL;
	}

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
		//dat->points[i].angle = hdr.angle*10 + hdr.span * i *10 / hdr.N;
		dat->points[i].degree = hdr.angle/10.0 + (hdr.span * i) / (10.0 * hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk && chk != sum)
	{
		delete dat;
		printf("chksum cf error");
		return NULL;
	}

	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get CF %d(%d) %d\n", hdr.angle, hdr.N, hdr.span);

	SetTimeStamp(dat);
	//dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static RawData* GetData0xDF(const RawDataHdr3& hdr, unsigned char* pdat, bool with_chk)
{
	RawData* dat = new RawData;

	if (!dat) {
		printf("out of memory\n");
		return NULL;
	}

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
		//dat->points[i].angle = hdr.first + dan * i;
		dat->points[i].degree = (hdr.first + dan * i) / 100.0;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk  && chk != sum)
	{
		delete dat;
		printf("chksum df error");
		return NULL;
	}

	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get DF %d=%d %d %d\n", hdr.angle, hdr.first, hdr.N, hdr.span);

	SetTimeStamp(dat);
	//dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;

}

static bool strange = false;
static int MsgProc(Parser* parser, int len, unsigned char* buf)
{
	if (len >= 8 && buf[0] == 'S' && buf[1] == 'T' && buf[6] == 'E' && buf[7] == 'D')
       	{
		parser->flags = update_flags(buf+2);
		return 1;
	}
		
	printf("unkown %d bytes %02x %02x %02x %02x\n", len, buf[0], buf[1], buf[2], buf[3]);
	strange = true;
	return 0;
}


static int ParseStream(Parser* parser, int len, unsigned char* buf, int* nfan, RawData* fans[]) 
{
	int idx = 0;
	int max_fan = *nfan;
	*nfan = 0;

	int unk = 0;
	unsigned char unknown[512];

	while (idx < len-32 && *nfan < max_fan)
	{
		unsigned char type = is_data(buf+idx);
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
		memcpy(&hdr, buf+idx, HDR_SIZE); 
		
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
				if (idx + HDR_SIZE + hdr.N*2+ 2 > len)
				{
					// need more bytes
					break;
				}
				
				RawData* fan = GetData0xCE_2(hdr, buf+idx,  parser->flags, parser->with_chk);
				if ( fan) 
				{
					fans[*nfan] = fan;
					*nfan += 1;
				}

				idx += HDR_SIZE + hdr.N*2 + 2;
			}
			else {
				if (idx + HDR_SIZE + hdr.N*3 + 2 > len)
				{
					// need more bytes
					break;
				}
				RawData* fan = GetData0xCE_3(hdr, buf+idx, parser->flags, parser->with_chk);
				if (fan) 
				{
					fans[*nfan] = fan;
					*nfan += 1;
				}
				idx += HDR_SIZE + hdr.N*3 + 2;
			}
			
		}
		else if (type == 0x99) {
			if (idx + hdr.N*3+ HDR99_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
		       	RawDataHdr99 hdr99;
			memcpy(&hdr99, buf+idx, HDR99_SIZE);

			RawData* fan = GetData0x99(hdr99, buf+idx, parser->with_chk);
			if (fan)
			{
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N*3+ HDR99_SIZE + 2;
		}

		else if (type == 0xCF) {
			if (idx + hdr.N*3+ HDR2_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
		       	RawDataHdr2 hdr2;
			memcpy(&hdr2, buf+idx, HDR2_SIZE);

			RawData* fan = GetData0xCF(hdr2, buf+idx, parser->with_chk);
			if (fan)
			{
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N*3+ HDR2_SIZE + 2;
		}
		else if (type == 0xC7) {
			if (idx + hdr.N*5+ HDR7_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
		       	RawDataHdr7 hdr7;
			memcpy(&hdr7, buf+idx, HDR7_SIZE);

			RawData* fan = GetData0xC7(parser, hdr7, buf+idx);
			if (fan)
			{
				//printf("set [%d] %d\n", *nfan, fan->angle);
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N*5 + HDR7_SIZE + 2;
		}
		else if (type == 0xDF) {
			if (idx + hdr.N*3+ HDR3_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
		       	RawDataHdr3 hdr3;
			memcpy(&hdr3, buf+idx, HDR3_SIZE);

			RawData* fan = GetData0xDF(hdr3, buf+idx, parser->with_chk);
			if (fan)
			{
				//printf("set [%d] %d\n", *nfan, fan->angle);
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N*3 + HDR3_SIZE + 2;
		} else {
			// should not 
		}
	} 
	return idx;
}


HParser ParserOpen(int raw_bytes, uint32_t device_ability, uint32_t init_states, 
		int init_rpm, double resample_res, bool with_chksum, uint32_t dev_id)
{
	Parser* parser = new Parser;

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

	return parser;
}

int ParserClose(HParser hP)
{
	Parser* parser = (Parser*)hP;
	
	while (parser->fan_segs != NULL) 
	{
		FanSegment* next = parser->fan_segs->next;
		delete parser->fan_segs;
		parser->fan_segs = next;
	}


	delete parser;
	return 0;
}

int ParserRunStream(HParser hP, int len, unsigned char* bytes, RawData* fans[]) 
{
	Parser* parser = (Parser*)hP;

	int nfan = MAX_FANS;

	unsigned char* buf = new unsigned char[len + parser->rest_len];
	if (!buf) {
		printf("out of memory\n");
		return -1;
	}

	if (parser->rest_len > 0) {
		memcpy(buf, parser->rest_buf, parser->rest_len);
	}
	memcpy(buf+parser->rest_len, bytes, len);
	len += parser->rest_len;

	strange = false;
	int used = ParseStream(parser, len, buf,  &nfan, fans);

#if 0
	for (int i=0; i<nfan; i++) 
	{
		fans[i]->ros_angle = LidarAng2ROS(fans[i]->angle + fans[i]->span);
	}
#endif
	if (strange) printf("len %d+%d used %d fan %d\n", len, parser->rest_len, used, nfan);

	parser->rest_len = len - used;
	if (parser->rest_len > 0) {
		memcpy(parser->rest_buf, buf+used, parser->rest_len);
	}

	delete[] buf;
	if (strange) printf("buf deleted\n");

	return nfan;
}

int ParserRun(HParser hP, int len, unsigned char* buf, RawData* fans[]) 
{	
	Parser* parser = (Parser*)hP;

	if ( *(uint32_t*)buf == 0x47534d4c) { //"LMSG"
		// skip all message
		return 0;
	}

	uint8_t type = buf[0];

	if (buf[1] != 0xfa) {
		//printf("skip packet %x %x len\n", buf[0], buf[1]);
	}
	else if (buf[0] == 0x88) 
	{ 
		PacketUart hdr;
		memcpy(&hdr, buf, sizeof(PacketUart));
		if (len >= hdr.len + sizeof(PacketUart) )
		{
			if (parser->dev_id != ANYONE && hdr.dev_id != parser->dev_id) { 
				// not my data
				return 0;
		       	}
			return ParserRunStream(hP, hdr.len, buf+sizeof(PacketUart), fans);
		}
	}
	else if (type == 0xCE)
	{
		RawDataHdr hdr;
		memcpy(&hdr, buf, HDR_SIZE); 

		if (HDR_SIZE + hdr.N*2+ 2 == len)
		{
			RawData* fan = GetData0xCE_2(hdr, buf,  parser->flags, parser->with_chk);
			if ( fan) 
			{
				fans[0] = fan;
				return 1;
			}
		}
		else if (HDR_SIZE + hdr.N*3 + 2 == len)
		{
			RawData* fan = GetData0xCE_3(hdr, buf, parser->flags, parser->with_chk);
			if (fan) 
			{
				fans[0] = fan;
				return 1;
			}
		}
		else printf("CE len %d N %d\n", len, hdr.N);
		
	}
	else if (type == 0xCF) 
	{
		RawDataHdr2 hdr;
		memcpy(&hdr, buf, HDR2_SIZE);

		if (hdr.N*3+ HDR2_SIZE + 2 > len)
		{
			// need more bytes
			printf("CF len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData* fan = GetData0xCF(hdr, buf, parser->with_chk);
		if (fan)
		{
			//printf("CF %d + %d %d\n", fan->angle, fan->span, fan->N);
			fans[0] = fan;
			return 1;
		}
	}
	else if (type == 0xDF) 
	{
		RawDataHdr3 hdr;
		memcpy(&hdr, buf, HDR3_SIZE);

		if (hdr.N*3+ HDR3_SIZE + 2 > len)
		{
			// need more bytes
			printf("DF len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData* fan = GetData0xDF(hdr, buf, parser->with_chk);
		if (fan)
		{
			//printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
	}
	else if (type == 0xC7) 
	{
		RawDataHdr7 hdr;
		memcpy(&hdr, buf, HDR7_SIZE);

		if (hdr.N*5+ HDR7_SIZE + 2 > len)
		{
			// need more bytes
			//printf("C7 len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData* fan = GetData0xC7(parser, hdr, buf);
		if (fan)
		{
			//printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
		return 0;
	}
	else if (type == 0x99) 
	{
		RawDataHdr99 hdr;
		memcpy(&hdr, buf, HDR99_SIZE);

		if (hdr.N*3+ HDR99_SIZE + 2 > len)
		{
			// need more bytes
			//printf("C7 len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData* fan = GetData0x99(hdr, buf, parser->with_chk);
		if (fan)
		{
			//printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
		return 0;
	}


	printf("skip packet %08x len %d\n", *(uint32_t*)buf, len);
	return 0;
}


char g_uuid[32] = "";

bool ParserScript(HParser hP, Script script, void* hnd)
{
	Parser* parser = (Parser*)hP;

	//read device's UUID 
	char buf[32];
	if (parser->device_ability & DF_WITH_UUID) {
		for (int i=0; i<10; i++)
		{
			if (script(hnd, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid))
			{
				printf("get product SN : \'%s\'\n", g_uuid);
				break;
			}
		}
	}

	// setup output data format
	if (parser->device_ability & DF_UNIT_IS_MM) {
		for (int i=0; i<10; i++) 
		{
			const char* cmd = (parser->init_states & DF_UNIT_IS_MM) ?  "LMDMMH" : "LMDCMH"; 
			if (script(hnd, 6, cmd, 10, "SET LiDAR ", 9, buf) )
			{
				printf("set LiDAR unit to %s\n", buf);
				break;
			}
		}
	}

	// enable/disable output intensity
	if (parser->device_ability & DF_WITH_INTENSITY) {
		for (int i=0; i<10; i++) 
		{
			const char* cmd = (parser->init_states & DF_WITH_INTENSITY) ? "LOCONH" : "LNCONH"; 
			if (script(hnd, 6, cmd, 6, "LiDAR ", 5, buf) )
			{
				printf("set LiDAR confidence to %s\n", buf);
				break;
			}
		}
	}


	// enable/disable shadow filter
	if (parser->device_ability & DF_DESHADOWED) {
		for (int i=0; i<10; i++) 
		{
			const char* cmd = (parser->init_states & DF_DESHADOWED) ? "LFFF1H" : "LFFF0H"; 
			if (script(hnd, 6, cmd, 6, "LiDAR ", 5, buf) )
			{
				printf("set LiDAR shadow filter to %s\n", buf);
				break;
			}
		}
	}


	// enable/disable smooth 
	if (parser->device_ability & DF_SMOOTHED) {
		for (int i=0; i<10; i++) 
		{
			const char* cmd = (parser->init_states & DF_SMOOTHED) ? "LSSS1H" : "LSSS0H"; 
			if (script(hnd, 6, cmd, 6, "LiDAR ", 5, buf) )
			{
				printf("set LiDAR smooth to %s\n", buf);
				break;
			}
		}
	}

	// setup rpm
	if (parser->init_rpm > 300 && parser->init_rpm < 3000) {
		for (int i=0; i<10; i++) 
		{
			char cmd[32];
			sprintf(cmd, "LSRPM:%dH", parser->init_rpm);
			if (script(hnd, strlen(cmd), cmd, 3, "RPM", 5, buf) )
			{
				printf("set RPM to %s\n", buf);
				break;
			}
		}
	}

	// set hard resample
	if (parser->device_ability & DF_WITH_RESAMPLE) {
		for (int i=0; i<10; i++) 
		{
			char cmd[32] = "LSRES:001H";
			if (parser->init_states & DF_WITH_RESAMPLE)
			{
				sprintf(cmd, "LSRES:%03dH", int(parser->resample_res*1000));
			}

			char pattern[] = "set resolution ";
			if (script(hnd, strlen(cmd), cmd, strlen(pattern), pattern, 1, buf) )
			{
				printf("set resolution %s\n", buf);
				break;
			}
		}
	}

	return true;
}
