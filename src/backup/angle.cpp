#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial/serial.h"
#include "std_msgs/Empty.h"
#include "campus_driving/INSPVAX.h"
#include <campus_driving/split.h>
#include <iostream>
#include <time.h>


using std::exception;
using namespace std;
serial::Serial ser; //声明串口对象

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CPT");
    ros::NodeHandle n;
    ros::Publisher ins_pub = n.advertise<campus_driving::INSPVAX>("cpt_ins",5);
    ros::Rate loop_rate(200);
    int count = 0;
    try
        {
        //设置串口属性，并打开串口
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
             ser.flush();  // 清空缓存
            // ref: https://github.com/wjwwood/serial/issues/148
            // ser.flushInput();  // 清空缓存
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
        //result.data = ser.read(ser.available());

        if(ser.available()){
            std::string str = ser.readline();
            std::string strID_ins = "%RB";
            //cout << str.empty() << endl;
            //cout << str <<endl;

            if(strncmp(str.c_str(), strID_ins.c_str() ,strID_ins.length()) == 0)
            {
                vector<string> temp;
                temp = split(str, ",");
                inspv.angle   =   atof(const_cast<const char *>(temp[1].c_str()));

                inspv.header.stamp = ros::Time::now();
                ins_pub.publish(inspv);
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
