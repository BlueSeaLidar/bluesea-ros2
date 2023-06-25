
#include "data_process.h"

void get_data(const sensor_msgs::msg::LaserScan& scan, int idx, double& dist, double& energy)
{
    if (idx < 0) 
        idx += scan.ranges.size();
    else if (idx >= scan.ranges.size()) 
        idx -= scan.ranges.size();
    dist = scan.ranges[idx];
    energy = scan.intensities[idx];
}

bool is_high(const sensor_msgs::msg::LaserScan& scan, int idx)
{
    double dist, energy;
    get_data(scan, idx, dist, energy);

    return dist < 1000000 && energy > 200;
}

bool data_process(sensor_msgs::msg::LaserScan& scan)
{
    int idx = 0;
    while (idx < scan.ranges.size())
    {
        if (scan.ranges[idx] < 10000000)
        {
            idx++;
            continue;
        }

        bool bl1 = is_high(scan, idx-1);
        bool bl2 = is_high(scan, idx-2);
        bool br1 = is_high(scan, idx+1);
        bool br2 = is_high(scan, idx+2);
        bool br3 = is_high(scan, idx+3);

        if ( (bl1 && br1 && br2) || (bl1 && bl2 && br1) )
        {
            double d1, d2, e1, e2;
            get_data(scan, idx-1, d1, e1);
            get_data(scan, idx+1, d2, e2);
            scan.ranges[idx] = (d1+d2)/2;
            scan.intensities[idx] = (e1+e2)/2;
            idx+=2;
        }
        else if (bl1 && bl2 && !br1 && br2 && br3)
        {
            double d1, d2, e1, e2;
            get_data(scan, idx-1, d1, e1);
            get_data(scan, idx+2, d2, e2);
            scan.ranges[idx] = (d1+d2)/2;
            scan.intensities[idx] = (e1+e2)/2;
            scan.ranges[idx+1] = (d1+d2)/2;
            scan.intensities[idx+1] = (e1+e2)/2;
            idx+=2;
        }
        else {
            idx++;
        }
        
    }
    return true;
}

int find(std::vector<RawData>a, int n, int x)
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
int autoGetFirstAngle(RawData raw, bool from_zero, std::vector<RawData> &raws,std::string &result)
{
    int angles = 0;
    int size = raws.size();
    //printf("angle %d  size:%d\n", raw.angle,size);
    if(size>=1)
    {
        RawData tmp = raws.at(size-1);
        RawData tmp2 = raws.at(0);
        if(raw.angle==tmp2.angle)
        {
            for (int i=0;i<size;i++)
            {
                tmp = raws.at(i);
                angles += tmp.span;
            }
            if (angles != 3600)
            {
                //result="angle sum "+std::to_string(angles);
                //printf("angle sum %d size:%d\n", angles,size);
                raws.clear();
                return -2;
            }
            else
            {
                int ret=-1;
                if(from_zero)
                     ret=find(raws,raws.size(),0);
                else
                     ret=find(raws,raws.size(),1800);

                raws.clear();
                return ret;
            }
        }
        if(raw.angle==(tmp.angle+tmp.span)%3600)
        {
            //说明是连续的扇区
            raws.push_back(raw);
        }

    }
    else
        raws.push_back(raw);


    return -1;

}

bool whole_data_process(RawData raw, bool from_zero,int collect_angle,std::vector<RawData>& whole_datas,PDataPoints *data,int &size,char *result)
{
    whole_datas.push_back(raw);
    if (raw.angle + raw.span == collect_angle*10)
    {

    }
    else
    {
        return false;
    }
    int count = 0, n = 0, angles = 0;
    for (std::vector<RawData>::iterator it = whole_datas.begin(); it != whole_datas.end(); ++it)
    {
        RawData tmp = *it;
        angles += tmp.span;
        count += tmp.N;
        n++;
    }
    if (angles != 3600)
    {
        sprintf(result,"angle sum:%d drop fans:%d  points:%d",angles,n,count);
        whole_datas.clear();
        return false;
    }
    size = sizeof(DataPoints)+sizeof(DataPoint)*count;
    *data=(PDataPoints)malloc(size);
    (*data)->N =count;
    (*data)->ts[0]=whole_datas.at(0).ts[0];
    (*data)->ts[1]=whole_datas.at(0).ts[1];

    int index=0;bool first=true;
    if(from_zero)
    {
        if(collect_angle>180)
        {
             index = (360-collect_angle)/((float)whole_datas.at(0).span/whole_datas.at(0).N);
             first=true;
        }
        else if(collect_angle<180)
        {
            index = (collect_angle-0)/((float)whole_datas.at(whole_datas.size()-1).span/whole_datas.at(whole_datas.size()-1).N);
            first=false;
        }
    }
    else
    {
        if(collect_angle>180)
        {
             index = (collect_angle-180)/((float)whole_datas.at(whole_datas.size()-1).span/whole_datas.at(whole_datas.size()-1).N);
             first=false;
        }
        else if(collect_angle<180)
        {
             index = (180-collect_angle)/((float)whole_datas.at(0).span/whole_datas.at(0).N);
             first=true;
        }
    }

   int pointindex=0;
   if(first)
   {
       for(int i=index;i<whole_datas.at(0).N;i++)
       {
           (*data)->points[pointindex]=whole_datas.at(0).points[i];
           pointindex++;
       }
       for(unsigned int i=1;i<whole_datas.size();i++)
       {
           for(unsigned int j=0;j<whole_datas.at(i).N;j++)
           {
               (*data)->points[pointindex]=whole_datas.at(i).points[j];
               pointindex++;
           }
       }
       for(int i=0;i<index;i++)
       {
           (*data)->points[pointindex]=whole_datas.at(0).points[i];
           pointindex++;
       }

   }
   else
   {
       for(int i=index;i<whole_datas.at(whole_datas.size()-1).N;i++)
       {
          (*data)->points[pointindex]=whole_datas.at(whole_datas.size()-1).points[i];
           pointindex++;
       }
       for(unsigned int i=0;i<whole_datas.size()-1;i++)
       {
           for(unsigned int j=0;j<whole_datas.at(i).N;j++)
           {
               (*data)->points[pointindex]=whole_datas.at(i).points[j];
               pointindex++;
           }
       }
       for(int i=0;i<index;i++)
       {
           (*data)->points[pointindex]=whole_datas.at(whole_datas.size()-1).points[i];
           pointindex++;
       }
   }
//    for(int i=0;i<pointindex;i++)
//    {
//        std::cout<<(*data)->points[i].angle<<(int)(*data)->points[i].confidence<<(*data)->points[i].distance<<std::endl;
//    }
    return true;

}