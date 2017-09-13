#ifndef BASE_CONTROL_H
#define BASE_CONTROL_H

#include <iostream>
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string.h> //for memset
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h> // for tf
#include <string> //for string
#include <math.h>

//速度发布50HZ的情况下，应低于20ms
#define TIME 19

#define DEVICE  "/dev/ttyUSB0"

#define BAUDRATE  115200

//轮子的间距m
#define WHEELS_SEPARATION 0.6

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

    string str;

    char rec_buf[100];
    
    geometry_msgs::Twist cmd_msg;

    ros::NodeHandle nh;

    ros::Publisher odom_pub;

    ros::Subscriber cmd_sub;

    ros::Time current_time, last_time;

    tf::TransformBroadcaster odom_broadcaster;

    int encoder_curr[2];

    int encoder_last[2];

    float angle_curr;

    float angle_last;

    bool FistTime;

    float x_pos;

    float y_pos;

    float th_pos;

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
