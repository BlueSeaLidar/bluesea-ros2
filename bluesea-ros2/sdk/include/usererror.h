#ifndef _USERERROR_H
#define _USERERROR_H

#include <string>
#include <map>
#include <cassert>

#define UNUSED(x) (void)(x)

class Error
{
public:
    Error(int value, const std::string& str)
    {
        m_value = value;
        m_message =    str;
#ifdef _DEBUG
        ErrorMap::iterator found = GetErrorMap().find(value);
        if (found != GetErrorMap().end())
            assert(found->second == m_message);
#endif
        GetErrorMap()[m_value] = m_message;
    }

    // auto-cast Error to integer error code
    operator int() { return m_value; }

private:
    int m_value;
    std::string m_message;

    typedef std::map<int, std::string> ErrorMap;
    static ErrorMap& GetErrorMap()
    {
        static ErrorMap errMap;
        return errMap;
    }

public:

    static std::string GetErrorString(int value)
    {
        ErrorMap::iterator found = GetErrorMap().find(value);
        if (found == GetErrorMap().end())
        {
            assert(false);
            return "";
        }
        else
        {
            return found->second;
        }
    }
};

static Error CMD_NO_ANSWER(-101,"send commands to lidar but no answer,please check the communication connection or retry");
static Error SOCKET_ERR(-102,"lidar connection abnormality, please check the hardware plugging or communication situation");
static Error LIDAR_NUM_LARGE(-103,"The number of lidars added exceeds the maximum limit");
static Error PORT_BIND_ERR(-104,"Port binding failed, occupied");
static Error COM_OPEN_ERR(-105,"Failed to open serial port, please check if it is occupied");
static Error CHANGE_BAUDRATE_ERR(-106,"Failure to modify serial port baud rate");
static Error GET_COM_ARG_ERR(-107,"Failure to get serial port parameters");
static Error SET_COM_ARG_ERR(-108,"Failure to modify serial port parameters");
static Error COM_READ_ERR(-109,"Failed to read data through serial port,please try after re-plugging the device");
static Error POINT_NUM_LARGE(-110,"The number of point clouds exceeds the maximum limit, please contact the developer");
static Error MEMORY_SPACE_ERR(-111,"Program application memory space is insufficient, please contact the developer");
static Error SAMPLE_TOO_SMALL(-112,"Soft Sampling Angle resolution too small, please reset");
static Error DISTANCE_ZERO_LARGE(-113,"Points with a distance of zero scale beyond the error_scale factor of the configuration file");
static Error DEVICE_OFFLINE(-114,"Serial device list does not exist, please check the device connection");
static Error CHECKSUM_ERR(-115,"Sector data calibration error, please check whether the data line is in accordance with the specification");
static Error TOPIC_NO_FIND(-116,"Find a topic that does not exist");
static Error NTP_IP_FORMAT_ERROR(-117,"ntp ip format set error");



static Error ALARM_LOWPOWER(-201,"Insufficient power supply, or occasional undervoltage");
static Error ALARM_ZERO_ERR(-202,"EnCoding disk zero detection error, please contact the developer");
static Error ALARM_CODEDISK_ERR(-203,"EnCoding disk error, please check if the radar is shaking badly or contact the developer");
static Error ALARM_RESET_ERR(-204,"Voltage reset failed, please try powering up again");
static Error ALARM_MOUDLE_ERR(-205,"Insufficient voltage or abnormal temperature for a long period of time");

static Error ALARM_BOTTOMPLATE_LOW_VOLTAGE(-206,"Bottom plate low voltage,please check the power supply or contact the developer");
static Error ALARM_BOTTOMPLATE_HIGH_VOLTAGE(-207,"Bottom plate high voltage,please check the power supply or contact the developer");
static Error ALARM_TEMPERATURE_ERR(-208,"Abnormal motor head temperature,please pay attention to the heat dissipation of the current working environment");
static Error ALARM_MOTERHEAD_LOW_VOLTAGE(-209,"Motor head low voltage,please check the power supply or contact the developer");
static Error ALARM_MOTERHEAD_HIGH_VOLTAGE(-210,"Motor head high voltage,please check the power supply or contact the developer");
//ERROR((int)LIDAR_NUM_LARGE,Error::GetErrorString(LIDAR_NUM_LARGE).c_str());

//log日志
#define __DEBUG__ 

#ifdef __DEBUG__  
#define DEBUG(format,...) \
{timespec spec;\
clock_gettime(CLOCK_REALTIME,&spec);\
printf("[INFO] [%ld.%09ld] [%s %d]: " format "\n",spec.tv_sec,spec.tv_nsec,__FUNCTION__,__LINE__,##__VA_ARGS__);}

#define ERROR(code,value) \
{timespec spec;\
clock_gettime(CLOCK_REALTIME,&spec);\
printf("\033[0m\033[1;31m[ERROR] [%ld.%09ld] [%s %d]: code:%d value:%s\033[0m\n",spec.tv_sec,spec.tv_nsec,__FUNCTION__,__LINE__,(int)code,(char*)value);}

#else  
#define DEBUG(format,...)  
#endif  


#endif // USERERROR_
