#include"alarm.h"
#include"algorithmAPI.h"
#include"parser.h"
#include"reader.h"
#include"usererror.h"


#define BLUESEA2_VERSION "2.6.1"

class BlueSeaLidarDriver
{
public:
    BlueSeaLidarDriver();
    ~BlueSeaLidarDriver();
    //Get the instructions that need to be executed for initialization
    void getInitCmds(ArgData &argdata);
    void openLidarThread();
    //void openHeartThread();

    bool sendCmd(std::string topic,std::string cmd,int proto);
    bool sendUdpCmd2(char *ip, int len, char *cmd);

    PubHub*getHub(int i);
    Parser*getParser(int i);
    bool checkIsRun(int i);
    int GetAllFans(PubHub* pub, ArgData argdata, int8_t &counterclockwise);
    bool GetFan(PubHub* pub, bool with_resample, double resample_res, RawData **fans);
    
    double ROSAng(double ang);
    int GetCount(std::vector<DataPoint> data, double min_deg, double max_deg, double &min_pos, double &max_pos);
private:

private:
    HReader m_reader;//NULL
    CommandList m_cmdlist;//init need run cmds
    //std::string m_type;//"uart"
    Parser *m_parsers[MAX_LIDARS];
	PubHub *m_hubs[MAX_LIDARS];
    ArgData m_argdata;
    int m_counterclockwise;
};
bool judgepcIPAddrIsValid(const char *pcIPAddr);