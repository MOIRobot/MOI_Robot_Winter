#ifndef BASE_CONTROL_H
#define BASE_CONTROL_H

#include <unistd.h>
#include <math.h>
#include <string.h> //for memset

#include <iostream>
#include <string> //for string

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h> // for tf

//脉冲数目转换为距离的系数
#define PULSE_K 10

//角度数据常数
#define ANGLE_K M_PI/(180.0*10)

using namespace std;
using namespace boost::asio;

enum Wheel{
    LeftWheel,
    RightWheel
};

class BaseControl{

private:

    /*服务*/
    io_service iosev;

    /*串口对象*/
    serial_port sp;

    /*定时器*/
    deadline_timer timer;

    ros::NodeHandle nh;

    ros::Publisher odom_pub;

    ros::Subscriber cmd_sub;

    ros::Time current_time, last_time;

    tf::TransformBroadcaster odom_broadcaster;
        
    geometry_msgs::Twist cmd_msg;

    string str;

    char rec_buf[100];

    int encoder_curr[2];

    int encoder_last[2];

    float angle_curr;

    float angle_last;

    bool FistTime;

    float x_pos;

    float y_pos;

    float th_pos;

    std::string usb_device_;

    //速度发布50HZ的情况下，应低于20ms
    //数据类型由launch文件决定，是std::string,double,int，不能用unsigned int ,float,etc
    int deadline_time_;

    int baudrate_;

    int pulse_per_rotation_;

    int ratio_;

    int loop_rate_;

    double wheel_radius_;

    double wheels_separation_;

public:

    /*构造函数*/
    BaseControl();

    ~BaseControl(){}

    /*初始化串口*/
    void InitSerial();

    /*设置速度*/
    void SetSpeed();

    /*获取推到位置姿态的数据（编码器脉冲数和角度）*/
    void GetPoseData();

    /*解析串口数据*/
    void ParseSerial();

    /*发布odom*/
    void PublishOdom();

    /*读取串口处理函数*/
    void HandleRead(boost::system::error_code ec, std::size_t bytes_transferred);

    void CmdCallback(const geometry_msgs::TwistConstPtr& cmd_msg);


};
#endif 
