#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial/serial.h"
#include "std_msgs/Empty.h"
#include "campus_driving/INSPVAX.h"
#include "campus_driving/BESTPOS.h"
#include <campus_driving/split.h>
#include <iostream>
#include <time.h>


using std::exception;
using namespace std;
typedef struct
{
    double x;
    double y;
}Position;
serial::Serial ser; //声明串口对象

//****** function section *********//
Position transCoordinate(double latitude,double longitude)
{
    double a_rad=6378137;
    double f=298.2572235635;
    double c_1,c_2,B_deg,L_deg,B,L,l_deg,l,t,u,A0,A2,A4,A6,A8,X,N,x,y,x_ori,y_ori,x_offset,y_offset;

    c_1=sqrt(1-pow(((f-1)/f),2));
    c_2=sqrt(pow((f/(f-1)),2)-1);


    B_deg = latitude; //30.51687345825
    L_deg = longitude; //114.34926179058
    B=B_deg*M_PI/180.0;
    L=L_deg;
    l_deg=L-(6.*20.-3.);
    l=l_deg*M_PI/180.0;
    t=tan(B);

    u=c_2*cos(B);
    A0=1+(3.0/4.0)*pow(c_1,2)+(45.0/64.0)*pow(c_1,4)+(350.0/512.0)*pow(c_1,6)+(11025.0/16384.0)*pow(c_1,8);
    A2=-1.0/2.0*((3.0/4.0)*pow(c_1,2)+(60.0/64.0)*pow(c_1,4)+(525.0/512.0)*pow(c_1,6)+(17640.0/16384.0)*pow(c_1,8));
    A4=1.0/4.0*((15.0/64.0)*pow(c_1,4)+(210.0/512.0)*pow(c_1,6)+(8820.0/16384.0)*pow(c_1,8));
    A6=-1.0/6.0*((35.0/64.0)*pow(c_1,6)+(2520.0/16384.0)*pow(c_1,8));
    A8=1.0/8.0*((315.0/16384.0)*pow(c_1,8));
    X=a_rad*(1-pow(c_1,2))*(A0*B+A2*sin(2*B)+A4*sin(4*B)+A6*sin(6*B)+A8*sin(8*B));
    N=a_rad/sqrt(1-pow(c_1,2)*pow(sin(B),2));
    y=X+(N*t*pow(l,2)*pow(cos(B),2))*(1.0/2.0+1.0/24.0*pow(l,2)*(5-pow(t,2)+9*pow(u,2)+4*pow(u,4))*pow(cos(B),2)+(1.0/720.0)*pow(l,4)*(61-58*pow(t,2)+pow(t,4)+270*pow(u,2)-330*pow(u,2)*pow(t,2))*pow(cos(B),4));
    x=500000+N*l*cos(B)*(1+(1.0/6.0)*pow(l,2)*pow(cos(B),2)*(1-pow(t,2)+pow(u,2))+(1.0/120.0)*pow(l,4)*pow(cos(B),4)*(5-18*pow(t,2)+pow(t,4)+14*pow(u,2)-58*pow(t,2)*pow(u,2)));
    x_ori = 245529.56042564771;  //车库前
    y_ori = 3380400.6653839252;   //车库前
    x_offset = x-x_ori; //正东
    y_offset = y-y_ori; ////正北

   // cout << "x,y " << x_offset << " " << y_offset << endl;
   // cout << "target" << "'xy', [2.079593675211072, 2.0639774608134758]" << endl;

    Position curPosition;
    curPosition.x = x_offset;
    curPosition.y = y_offset;
    return curPosition;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CPT");
    ros::NodeHandle n;
    ros::Publisher ins_pub = n.advertise<campus_driving::INSPVAX>("cpt_ins",5);
    ros::Publisher bestp_pub = n.advertise<campus_driving::BESTPOS>("cpt_bestp",5);
    ros::Publisher corr_pub = n.advertise<campus_driving::INSPVAX>("cpt_corr",5);
    ros::Rate loop_rate(200);
    int count = 0;
    try
        {
        //设置串口属性，并打开串口
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(38400);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
            // ser.flush();  // 清空缓存
            // ref: https://github.com/wjwwood/serial/issues/148
            ser.flushInput();  // 清空缓存
        }
    catch (exception &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    while(ros::ok())
    {
        clock_t start = clock();
        //reserved,处理ROS的信息，比如订阅消息,并调用回调函数
        //所接到的订阅消息并不是立刻就被处理(执行回调函数)，
        //而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用
        //difference between spin and spinOnce: http://www.cnblogs.com/liu-fa/p/5925381.html
        ros::spinOnce();
        //cout.precision(16);
        std_msgs::String msg;
        std::stringstream ss;
        campus_driving::INSPVAX inspv;
        campus_driving::BESTPOS bestp;
        campus_driving::INSPVAX corr;
        //result.data = ser.read(ser.available());

        if(ser.available()){
            std::string str = ser.readline();
            std::string strID_ins = "%INSPVASA";
            std::string strID_best = "#BESTPOSA";
            std::string strID_corr = "%CORRIMUDATASA";
            //cout << str.empty() << endl;
            //cout << str <<endl;

            if(strncmp(str.c_str(), strID_ins.c_str() ,strID_ins.length()) == 0)
            {
                vector<string> temp,statusTemp2;
                temp = split(str, ",");
                inspv.latitude   =   atof(const_cast<const char *>(temp[4].c_str()));
                inspv.longitude   =   atof(const_cast<const char *>(temp[5].c_str()));
                inspv.altitude   =   atof(const_cast<const char *>(temp[6].c_str()));
                inspv.north_velocity   =   atof(const_cast<const char *>(temp[7].c_str()));
                inspv.east_velocity   =   atof(const_cast<const char *>(temp[8].c_str()));
                inspv.up_velocity   =   atof(const_cast<const char *>(temp[9].c_str()));
                inspv.roll   =   atof(const_cast<const char *>(temp[10].c_str()));
                inspv.pitch   =   atof(const_cast<const char *>(temp[11].c_str()));
                inspv.azimuth   =   atof(const_cast<const char *>(temp[12].c_str()));

                std::string statusTemp1 = temp[13];
                statusTemp2 = split(statusTemp1,"*");
                inspv.ins_status = statusTemp2[0];
                Position curPosition;
                curPosition = transCoordinate(inspv.latitude,inspv.longitude);
                inspv.x = curPosition.x;
                inspv.y = curPosition.y;
                inspv.header.stamp = ros::Time::now();
                ins_pub.publish(inspv);
            }
            if(strncmp(str.c_str(), strID_best.c_str() ,strID_best.length()) == 0)
            {
                vector<string> temp;
                temp = split(str, ",");
                bestp.position_type = temp[10];

                bestp_pub.publish(bestp);
            }
            if(strncmp(str.c_str(), strID_corr.c_str() ,strID_corr.length()) == 0)
            {
                vector<string> temp;
                temp = split(str, ",");
                corr.w_x = atof(const_cast<const char *>(temp[4].c_str()));
                corr.w_y = atof(const_cast<const char *>(temp[5].c_str()));
                corr.w_z = atof(const_cast<const char *>(temp[6].c_str()));
                corr.a_x = atof(const_cast<const char *>(temp[7].c_str()));
                corr.a_y = atof(const_cast<const char *>(temp[8].c_str()));
                corr.a_z = atof(const_cast<const char *>(temp[9].c_str()));

                corr_pub.publish(corr);
            }
        }

        //cout << "tlq: " << inspv.longitude<< endl;
        //msg.data = ss.str();
        //ROS_INFO("%s", msg.data.c_str());
        //ROS_WARN("Necessary topics are not subscribed yet ... ");

        clock_t ends = clock();
        //cout <<"Running Time : "<<(double)(ends - start)*1000./ CLOCKS_PER_SEC << endl;
        //ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    return 0;
}
