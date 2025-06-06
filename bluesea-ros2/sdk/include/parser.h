#ifndef _PARSER_
#define _PARSER_
#include <arpa/inet.h>
#include <string>
#include<iostream>
#include<list>
#include<queue>
#include <unistd.h>
#include <fcntl.h>
#include"common_data.h"
#include"usererror.h"
#include"alarm.h"

Parser* ParserOpen(int raw_bytes, bool with_chksum,int dev_id,
				   int error_circle, double error_scale, bool from_zero,int time_mode,CommandList cmd, char *ip, int port);

int strip(const char *s, char *buf);
//int ParserClose(HParser);
int ParserRunStream(Parser*, int len, unsigned char *buf, RawData *fans[]);
int ParserRun(LidarNode, int len, unsigned char *buf, RawData *fans[]);

void SetTimeStamp(RawData *);
//void saveLog(const char *logPath, int type, const char *ip, const int port, const unsigned char *buf, unsigned int len);
int mkpathAll(std::string s, mode_t mode);
unsigned int stm32crc(unsigned int *ptr, unsigned int len);
int alarmProc(unsigned char *buf, int len);
//int autoGetFirstAngle(RawData raw, bool from_zero, std::vector<RawData> &raws,std::string &result);

std::string stringfilter(char *str, int num);
int getFirstidx(RawData raw, int angle);
void timestampMode(int type);

//中断进程
void closeSignal(int sig);
// //指定编号雷达发送指令
// bool SendCmd(int len, char *cmd, int index);
// //获取一帧第一个点所在的扇区，以及该点所在扇区的位置
// bool autoGetFirstAngle2(PubHub* pub, bool from_zero);
//对读取的数据进行解析    参数:实际获取的数据    配置文件对应的参数
//int GetAllFans(HPublish pub,ArgData argdata,uint8_t &counterclockwise);

void PublishData(PubHub* pub, int n, RawData **fans);

void redirect_stdout_to_log(const char *path) ;

#endif
