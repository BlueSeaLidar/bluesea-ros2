#ifndef _PARSER_
#define _PARSER_

#define DF_UNIT_IS_MM 		0x0001
#define DF_WITH_INTENSITY 	0X0002
#define DF_DESHADOWED		0x0004
#define DF_SMOOTHED		0x0008
#define DF_FAN_90		0x0020
#define DF_WITH_RESAMPLE	0X0080
#define DF_MOTOR_REVERSE	0x0100
#define DF_WITH_UUID		0X1000

#define EF_ENABLE_ALARM_MSG	0X10000

#define MAX_FANS 120

#define MAX_POINTS 1000

//#define ANYONE 0x1234abcd
#define ANYONE -1


#include <arpa/inet.h>
struct DataPoint
{
	uint16_t idx;
	// int angle;
	double degree;
	uint16_t distance; // mm
	uint8_t confidence; 
};

struct RawData
{
	unsigned short code;
	unsigned short N;
	unsigned short angle; // 0.1 degree
	unsigned short span; // 0.1 degree
	unsigned short fbase;
	unsigned short first;
	unsigned short last;
	unsigned short fend;
	// short ros_angle;	// 0.1 degree
	DataPoint points[MAX_POINTS];
	uint32_t ts[2];
	uint8_t	counterclockwise;

	//unsigned short distance[1000];
	//unsigned char confidence[1000];
	//float angles[1000];
};

typedef void* HParser;
typedef void* HReader;
typedef void* HPublish;


struct LidarNode 
{
	HParser hParser;
	HPublish hPublish;
	char ip[30];
	int port;
	in_addr_t s_addr;
};


HParser ParserOpen(int raw_bytes, 
		uint32_t device_ability,
	       	uint32_t flags, 
		int init_rpm,
		double resample_res,
	       	bool with_chk, 
		uint32_t dev_id);

typedef bool (*Script)(void*, int cmd_len, const char* cmd_str, 
		int pattern_len, const char* pattern_str, 
		int nfetch, char* fetch);
bool ParserScript(HParser, Script, void*);

int ParserClose(HParser);
int ParserRunStream(HParser, int len, unsigned char* buf, RawData* fans[]);
int ParserRun(LidarNode, int len, unsigned char* buf, RawData* fans[]);

void SetTimeStamp(RawData*); 

extern char g_uuid[32];


#endif
