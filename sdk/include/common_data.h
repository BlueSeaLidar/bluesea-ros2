#ifndef _COMMON_DATA_
#define _COMMON_DATA_

#include<stdint.h>
#include<stdio.h>
#include<iostream>
#include<pthread.h>
#include<vector>
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
#define MAX_POINTS 510
#define ANYONE -1
#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16
#define HDR7_SIZE 28
#define HDR99_SIZE 32
#define HDRAA_SIZE 48
#define BUF_SIZE 8 * 1024
#define MAX_LIDARS 8
#define GS_PACK 0x4753
#define S_PACK 0x0053
#define C_PACK 0x0043
#define M_PI 3.1415926535898
#define getbit(x, y) ((x) >> (y)&1) // 获取X的Y位置数据
#define setbit(x, y) x |= (1 << y)	// 将X的第Y位置1
#define clrbit(x, y) x &= ~(1 << y) // 将X的第Y位清0


struct DataPoint
{
	//uint16_t idx;
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
	int8_t counterclockwise;
	uint32_t flags;	//消息类型
};
struct AllPointData
{
	std::vector<DataPoint>points;//已缓存的所有数据
	std::vector<uint32_t>timestamp;//时间戳[2]

};

struct EEpromV101
{
	char label[4];			// "EPRM"
    uint16_t pp_ver; // 协议版本
    uint16_t size;			// 结构大小

	//uint32_t version;		// firmware version

	// device
    uint8_t dev_sn[20];//sn号
    uint8_t dev_type[16];//型号
    uint32_t dev_id;		// 编号

	// network
	uint8_t IPv4[4];
	uint8_t mask[4];
	uint8_t gateway[4];
    uint8_t srv_ip[4];//上传IP
    uint16_t srv_port;//上传端口
    uint16_t local_port;//本地端口

	//
	uint16_t RPM;
    uint16_t zone_dist_thr;/* 防区连续检测距离阈值 */
    uint8_t fir_filter;//防区报警点数标准
    uint8_t cir;//防区报警圈数过滤标准
	uint16_t with_resample;

    uint8_t auto_start;//开机自动旋转
    uint8_t target_fixed;//固定上传
    uint8_t with_smooth;//数据平滑
    uint8_t with_filter;//去拖点


    int16_t dist_comp_coeff_a;
    int16_t dist_comp_coeff_b;
    uint16_t dist_comp_range_1;
    uint16_t dist_comp_range_2;
    uint32_t net_watchdog;//看门狗

    uint32_t pnp_flags;//PNP/NPN
    uint16_t deshadow;//平滑系数
    uint8_t zone_acted;//激活防区
    uint8_t should_post;//上传方式   0 无数据 1仅数据 2报警 3报警+数据

    uint8_t functions_map[16];//I/O输入输出口
    int16_t zero_pos; //零位校正
    uint16_t wlp_magic; // 无线
    uint16_t wlp_pwm; // 无线供电
    uint8_t motor_ccw; // 旋转方向
    uint8_t reserved[29];

};



typedef void *HReader;
typedef void *HPublish;


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
struct RawDataHdrAA {
    uint16_t code; // 0xFAAA
    uint16_t N;
    uint16_t whole_fan;
    uint16_t ofset;
    uint32_t beg_ang;
    uint32_t end_ang;
    uint32_t flags;
    uint32_t second;
    uint32_t nano_sec;
    uint32_t dev_id;
    uint32_t reserved[4];
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

struct FanSegment_C7
{
	RawDataHdr7 hdr;

	uint16_t dist[MAX_POINTS];
	uint16_t angle[MAX_POINTS];
	uint8_t energy[MAX_POINTS];

	struct FanSegment_C7 *next;
};
struct FanSegment_AA
{
    RawDataHdrAA hdr;			//CN：HdrAA结构体		EN：Hdr7structure

    uint16_t dist[MAX_POINTS];	//CN:距离				EN:distance
    uint16_t angle[MAX_POINTS]; //CN:角度				EN:angle
    uint8_t energy[MAX_POINTS]; //CN:能量强度			EN:energy intensity

    struct FanSegment_AA* next;	// CN:下个扇区指针		EN:next sector pointer
};
struct CommandList
{
	char  uuid[12];
	char  model[12];
	char  rpm[12];
	char  res[12];
	char  smooth[12];
	char fitter[12];
	char confidence[12];
	char unit_mm[12];
	
	char alarm[12];
	char direction[12];
	char ats[12];


};
struct Parser
{
	int rest_len;
	unsigned char rest_buf[BUF_SIZE];
	bool with_chk;
	int raw_mode;
	uint32_t dev_id;
	uint32_t flags;
	FanSegment_AA *fan_segs_aa;
	FanSegment_C7 *fan_segs_c7;
	int error_circle;
	float error_scale;
	bool is_from_zero;
	int time_mode;
	CommandList cmd;
	char ip[16];
	int port;
	bool direction;
	bool isrun;
};
struct PubHub
{
	pthread_mutex_t mtx;
	int nfan;
	RawData *fans[MAX_FANS];
	int error_num;
	int offsetangle;
	int offsetidx;
	std::vector<DataPoint>consume;//统计完剩余的点数
	uint32_t ts_beg[2];
	uint32_t ts_end[2];
};

struct LidarNode
{
	Parser* hParser;
	PubHub* hPublish;
	char ip[16];
	int port;
};
struct UartState
{
    //byte1
    bool unit_mm;//0 cm 1 mm
    bool with_conf;//0 close 1 open
    bool with_smooth;
    bool with_fitter;
    bool span_9;
    bool span_18;
    bool span_other;
    bool resampele;//重采样
    //byte2

    bool moter_turn;//0正向 1反向
    bool span_8;
    bool span_16;
    //byte3
    bool byte3_error0;
    bool byte3_error1;
    bool byte3_error2;
    bool byte3_error3;
    //byte4
    bool byte4_error0;
    bool byte4_error1;
    bool byte4_error2;
    bool byte4_error3;
    bool byte4_error4;
    bool byte4_error5;
    bool byte4_error6;
    bool byte4_error7;
};
typedef struct
{
    unsigned short N;
    uint32_t ts[2];

    DataPoint points[0];

}DataPoints,*PDataPoints;

struct UartInfo
{
	char type[8]; // uart or vpc
	int fd_uart;
	char port[128];
	int baudrate;
	int *rate_list;

	Parser* hParser;
	PubHub* hPublish;
	pthread_t thr;
};

struct UDPInfo
{
	char type[8]; //"udp"
	int nnode;
	LidarNode lidars[MAX_LIDARS];

	int fd_udp;
	int listen_port;
	bool is_group_listener;
	pthread_t thr;

};

struct ScriptParam
{
	UDPInfo *info;
	int id;
};

struct KeepAlive
{
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};

struct LidarInfo {
	PubHub* pub;
	Parser* parser;
	char lidar_ip[32];
	int lidar_port;
};

struct ConnectArg
{
	std::string laser_topics;
	std::string cloud_topics;
	std::string arg1;
	int arg2;
};
struct Range
{
	double min;
	double max;
};
//定制隐藏功能
struct Custom
{
	int error_circle;
	double error_scale;
	bool is_group_listener;
	std::string group_ip;
};
//过滤算法
struct Fitter
{
	bool isopen;
	int type;
	double max_range;
	double min_range;
	double max_range_difference;
	int filter_window;

};
struct ArgData
{
	std:: string frame_id;
	int dev_id;
	std::vector<ConnectArg>connectargs;
	int raw_bytes;
	bool from_zero;
	bool output_scan;
	bool output_cloud;
	bool inverted;
	bool reversed;
	bool hard_resample;
	bool soft_resample;
	bool with_angle_filter;
	double min_angle;
	double max_angle;
	bool output_360;
	double min_dist;
	double max_dist;
	std::vector<Range>masks;
	std::string type;
	int localport;
	int num;
	int uuid;
	int rpm;
	int model;
	double resample;
	int with_smooth;
	int with_deshadow;
	int alarm_msg;
	int direction;
	int unit_is_mm;
	int with_confidence;
	int ats;
	int time_mode;
	Fitter fitter;
	Custom custom;

};
typedef struct
{
  char sign[2];  /* AM */
  uint8_t dev_id;  /* 设备序号 */
  uint32_t timestamp;  /* 时间戳 */
  uint16_t id;  /* 消息序号 */
  uint32_t events;  /* 报警事件 */
  uint32_t zone_actived;  /* 当前激活防区 */
  uint8_t reserved[11];
  uint32_t crc;
}PROCOTOL_HOST_ALARM_ST;


#endif