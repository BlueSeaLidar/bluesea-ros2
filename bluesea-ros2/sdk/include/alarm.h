#ifndef _ALARM_
#define _ALARM_ 

#define LMSG_ERROR	1
#define LMSG_INFO	2
#define LMSG_DEBUG	4
#define LMSG_ALARM	0x100


#define LERR_LOW_POWER		1
#define	LERR_MOTOR		2
#define LERR_RANGER_HI		4
#define LERR_NETWORK 		8
#define LERR_RANGER_IDLE	0x10
#define LERR_ZERO_MISS		0x20
#include<stdint.h>
struct LidarMsgHdr
{
	char sign[4];  // must be "LMSG"
	uint32_t proto_version; // 0x101
	char dev_sn[20];
	uint32_t dev_id; //
	uint32_t timestamp;//
	uint32_t type;
	uint32_t data;
	uint16_t id;
	uint16_t extra;	
};

struct LidarAlarm
{
	LidarMsgHdr hdr;
	uint32_t zone_actived;
	uint8_t all_states[32];
	uint32_t reserved[11];
};

#endif
