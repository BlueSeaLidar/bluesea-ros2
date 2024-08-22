#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_srvs/srv/empty.hpp>
#include <cstdlib>
#include "../sdk/include/reader.h"
#include "base/srv/control.hpp"

bool g_isrun = false;
pthread_t g_threadid;
void getTime_HMS(char *data);
void *HeartThreadProc(void*p);

bool heart_motor(const std::shared_ptr<base::srv::Control::Request> req, std::shared_ptr<base::srv::Control::Response> res)
{
    UNUSED(res);
    DEBUG("HeartCheck change status:%d\n:", req->flag);
    if(req->flag==g_isrun)
        return true;
    else
    {
        if(req->flag) 
            pthread_create(&g_threadid, NULL, HeartThreadProc, NULL);
        
        g_isrun = req->flag;
    }
    return true;
}
using std::placeholders::_1;
using std::placeholders::_2;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bluesea_node");
	rclcpp::QoS qos(0);
	qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    rclcpp::Service<base::srv::Control>::SharedPtr service = node->create_service<base::srv::Control>("/heart_motor", heart_motor);

    DEBUG("bluesea2_heart_check into\n");
    rclcpp::spin(node);
    return 0;
}
void getTime_HMS(char *data)
{
    time_t t0 = time(NULL);
    int hh = t0 % (3600 * 24) / 3600;
    int mm = t0 % 3600 / 60;
    int ss = t0 % 60;
    sprintf(data, "%d-%d-%d", hh, mm, ss);
}
// 打印具体信息
void  HeartMsg(char *buf_data, int buf_len)
{

    if (buf_len == sizeof(DevInfoV101))
    {
        DevInfoV101 *dvi = (DevInfoV101 *)buf_data;
        DEBUG("timestamp:%u.%u  sn:%s model:%s id:%d rpm:%.1f", dvi->timestamp[0], dvi->timestamp[1], dvi->dev_sn, dvi->dev_type, dvi->dev_id, dvi->rpm / 10.0);
        char tmp_IPv4[16] = {0};
        char tmp_mask[16] = {0};
        char tmp_gateway[16] = {0};
        char tmp_srv_ip[16] = {0};

        sprintf(tmp_IPv4, "%d.%d.%d.%d", dvi->ip[0], dvi->ip[1], dvi->ip[2], dvi->ip[3]);
        sprintf(tmp_mask, "%d.%d.%d.%d", dvi->mask[0], dvi->mask[1], dvi->mask[2], dvi->mask[3]);
        sprintf(tmp_gateway, "%d.%d.%d.%d", dvi->gateway[0], dvi->gateway[1], dvi->gateway[2], dvi->gateway[3]);
        sprintf(tmp_srv_ip, "%d.%d.%d.%d", dvi->remote_ip[0], dvi->remote_ip[1], dvi->remote_ip[2], dvi->remote_ip[3]);
        DEBUG("lidar_ip:%s lidar_mask:%s lidar_gateway:%s  lidar_port:%d upload_ip:%s  upload_port:%d", tmp_IPv4, tmp_mask, tmp_gateway, dvi->port, tmp_srv_ip, dvi->remote_udp);
        std::string alarmMsg;
        if (dvi->alarm[FUN_LV_1])
            alarmMsg += " Observation Zone";
        if (dvi->alarm[FUN_LV_2])
            alarmMsg += " Alert Zone";
        if (dvi->alarm[FUN_LV_3])
            alarmMsg += " Alarm Zone";
        if (dvi->alarm[FUN_COVER])
            alarmMsg += " Masking";
        if (dvi->alarm[FUN_NO_DATA])
            alarmMsg += " No data";
        if (!dvi->alarm[FUN_ZONE_ACTIVE])
            alarmMsg += " No zone active";
        if (dvi->alarm[FUN_SYS_ERR])
            alarmMsg += " System error";
        int v = dvi->status & 0x7;
        if (v == 1 || v == 3)
        {
            if (!dvi->alarm[FUN_NET_LINK])
                alarmMsg += " NET disconnected";
        }
        else if (v == 2)
        {
            if (!dvi->alarm[FUN_USB_LINK])
                alarmMsg += " USB disconnected";
        }
        if (dvi->alarm[FUN_UPDATING])
            alarmMsg += " System Upgrade";
        if (dvi->alarm[FUN_ZONE_DEFINED])
            alarmMsg += " Zone not defined";

        if(!alarmMsg.empty())
            DEBUG("lidar_alarm:%s",alarmMsg.c_str());
    }

}

void *HeartThreadProc(void*p)
{
    UNUSED(p);
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    int yes = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes)) < 0)
    {
        return NULL;
    }
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HeartPort);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int iResult = ::bind(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (iResult != 0)
        return NULL;

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
    {
        return NULL;
    }
    while (g_isrun)
    {
        socklen_t sz = sizeof(addr);
        char raw[4096];
        int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr *)&addr, &sz);
        if (memcmp(raw, "LiDA", 4) == 0)
        {
            HeartMsg(raw, dw);
        }
    }
    close(sock);
    return NULL;
}