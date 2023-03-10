#ifndef _PARSER_
#define _PARSER_
#include <arpa/inet.h>
#include <string>
#define DF_UNIT_IS_MM 0x0001
#define DF_WITH_INTENSITY 0X0002
#define DF_DESHADOWED 0x0004
#define DF_SMOOTHED 0x0008
#define DF_FAN_90 0x0020
#define DF_WITH_RPM 0X0040
#define DF_WITH_RESAMPLE 0X0010
#define DF_WITH_RESAMPLE_SOFT 0X0080
#define DF_MOTOR_REVERSE 0x0100
#define DF_WITH_UUID 0X1000

#define EF_ENABLE_ALARM_MSG 0X10000

#define MAX_FANS 120

#define MAX_POINTS 1000

// #define ANYONE 0x1234abcd
#define ANYONE -1

#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16
#define HDR7_SIZE 28
#define HDR99_SIZE 32
#define BUF_SIZE 8 * 1024

#define getbit(x, y) ((x) >> (y)&1) // 获取X的Y位置数据
#define setbit(x, y) x |= (1 << y)	// 将X的第Y位置1
#define clrbit(x, y) x &= ~(1 << y) // 将X的第Y位清0

struct DataPoint
{
	uint16_t idx;
	// int angle;
	double degree;
	uint16_t distance; // mm
	uint8_t confidence;
};
struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};

struct RawData
{
	unsigned short code;
	unsigned short N;
	unsigned short angle; // 0.1 degree
	unsigned short span;  // 0.1 degree
	unsigned short fbase;
	unsigned short first;
	unsigned short last;
	unsigned short fend;
	// short ros_angle;	// 0.1 degree
	DataPoint points[MAX_POINTS];
	uint32_t ts[2];
	uint8_t counterclockwise;

	// unsigned short distance[1000];
	// unsigned char confidence[1000];
	// float angles[1000];
};
struct EEpromV101
{
	char label[4];	 // "EPRM"
	uint16_t pp_ver; // paramter protocol version
	uint16_t size;	 // total size of this structure

	// uint32_t version;		// firmware version

	// device
	uint8_t dev_sn[20];
	uint8_t dev_type[16];
	uint32_t dev_id; // identiy

	// network
	uint8_t IPv4[4];
	uint8_t mask[4];
	uint8_t gateway[4];
	uint8_t srv_ip[4];
	uint16_t srv_port;
	uint16_t local_port;

	//
	uint16_t RPM;
	uint16_t RPM_pulse;
	uint8_t fir_filter;
	uint8_t cir;
	uint16_t with_resample;

	uint8_t auto_start;
	uint8_t target_fixed;
	uint8_t with_smooth;
	uint8_t with_filter;

	//
	uint8_t ranger_bias[8];
	uint32_t net_watchdog;

	uint32_t pnp_flags;
	uint16_t deshadow;
	uint8_t zone_acted;
	uint8_t should_post;

	uint8_t functions_map[16];
	uint8_t reserved[36];
};
typedef void *HParser;
typedef void *HReader;
typedef void *HPublish;

struct LidarNode
{
	HParser hParser;
	HPublish hPublish;
	char ip[30];
	int port;
	in_addr_t s_addr;
};
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

struct RawDataHdr7
{
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

struct RawDataHdr99
{
	uint16_t code;
	uint16_t N;
	uint16_t from;
	uint16_t total;
	uint32_t flags;
	uint32_t timestamp;
	uint32_t dev_no;
	uint32_t reserved[3];
};

typedef struct
{
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

	struct FanSegment *next;
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

	FanSegment *fan_segs;
	int error_circle;
	float error_scale;
};

HParser ParserOpen(int raw_bytes, uint32_t device_ability,uint32_t flags,int init_rpm,
				   double resample_res,bool with_chk,uint32_t dev_id,int error_circle,double error_scale);

typedef bool (*Script)(void *, int cmd_len, const char *cmd_str,
					   int pattern_len, const char *pattern_str,
					   int nfetch, char *fetch);

typedef bool (*S_PACK)(void *, int cmd_len, const char *cmd_str, void *);
bool ParserScript(HParser, Script, S_PACK, const char *type, void *);

int ParserClose(HParser);
int ParserRunStream(HParser, int len, unsigned char *buf, RawData *fans[]);
int ParserRun(LidarNode, int len, unsigned char *buf, RawData *fans[]);

void SetTimeStamp(RawData *);
void saveLog(bool isSaveLog, const char *logPath, int type, const char *ip, const int port, const unsigned char *buf, unsigned int len);
int mkpathAll(std::string s, mode_t mode);
unsigned int stm32crc(unsigned int *ptr, unsigned int len);
int alarmProc(unsigned char *buf, int len);
extern char g_uuid[32];

#endif
