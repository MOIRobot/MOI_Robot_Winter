/*
g++ base_control.cpp -lboost_system

通过ROS参数设置：
设备名称：/dev/USB0
波特率：115200

通讯协议：服务机器人通信格式_4.xlsx
*/

/*
本来是24的字节的数据，可是有时候会读到3+21,有时候会读到0+24,有时候是12+12,等等，我感觉是不是timeout了，然后放弃读后面的数据了，
我通过将一个USB串口的TX和RX连接到一起，然后调整timeout的时间，发现计算是将时间间隔调整到1ms，也不会出现读到0bytes的情况，都是稳定的，都是15bytes，
我的发送频率是20ms间隔，因此timeout最大只能设置为19ms，大于19ms的时候会出现两次一次读的情况30bytes


*/
#include "winter_bringup/base_control.h"

BaseControl::BaseControl():
                  sp(iosev),
               timer(iosev),
              FistTime(true)
{
  ros::NodeHandle private_nh("~");

  //<node_handle.h line:2038>
  private_nh.param("usb_device", usb_device_, std::string("/dev/ttyUSB0"));
  private_nh.param("deadline_time", deadline_time_, 1);
  private_nh.param("baudrate", baudrate_, 115200);
  private_nh.param("wheels_separation", wheels_separation_, 0.6);
  private_nh.param("wheel_radius", wheel_radius_, 0.06);
  private_nh.param("pulse_per_rotation", pulse_per_rotation_, 500);
  private_nh.param("ratio", ratio_, 25);
  private_nh.param("loop_rate", loop_rate_, 50);
  

  cmd_sub = nh.subscribe("/smooth_cmd_vel", 1, &BaseControl::CmdCallback, this);

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

  memset(rec_buf, 0, sizeof(rec_buf));

  ros::Rate loop_rate(loop_rate_);

  try{

    InitSerial();

    while(ros::ok()){

        GetPoseData();

        SetSpeed();   
        //执行所有准备好的handler
        size_t handler_num = iosev.poll();///区别poll_one() usr/include/boost/asio/io_service.hpp
        // cout << "handler_num: " << handler_num << endl;

        ParseSerial();                          

        //PublishOdom();

        ros::spinOnce();

        loop_rate.sleep();

        //usleep(20000);
    }

    sp.close();

  }catch(std::exception& e){
      cout << e.what() << endl;
  }

}

void BaseControl::InitSerial()
{

    sp.open(usb_device_);
    sp.set_option(serial_port::baud_rate(baudrate_));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));
    
    if(sp.is_open())
        cout << "Open Serial Succeed" << endl;
    else
        cout << "Open Serial Error" << endl;
    
    //读数据
    //boost::asio::read_until(sp, buffer(rec_buf), '#');
    //read_until(sp, buffer(rec_buf), '#');
    //boost::asio::async_read(sp, buffer(rec_buf), boost::bind(&BaseControl::HandleRead, this,  _1, _2));

}

void BaseControl::HandleRead(boost::system::error_code ec, std::size_t bytes_transferred)
{
    //cout.write(rec_buf, bytes_transferred);
    for(int i = 0 ; i < bytes_transferred; i++)
        printf("%x ",rec_buf[i]);
    cout<<"--Read " << bytes_transferred << " bytes" << endl;

    bytes_read = bytes_transferred;

    //boost::asio::async_read_until(sp, buffer(rec_buf), '#', boost::bind(&BaseControl::HandleRead, this,  _1, _2));
    boost::asio::async_read(sp, buffer(rec_buf), boost::bind(&BaseControl::HandleRead, this,  _1, _2));
}

void BaseControl::ParseSerial()
{
    //处理接受到的数据
    //std::size_t recv_size = sizeof(rec_buf);

    //Pose Data数据
    if((rec_buf[0] == 0x45) && (rec_buf[1] == 0x54) && (rec_buf[2] == 0x09) && (rec_buf[3] == 'G') && (rec_buf[4] == 'P')){
        //check sum
        /*if((((rec_buf[5] + rec_buf[6] + rec_buf[7] + rec_buf[8] + rec_buf[9] + rec_buf[10]) & 0x3F) + 0x30) == rec_buf[11]){
            velocity[LeftWheel] = (rec_vec[5] << 8) | rec_vec[6];
            velocity[RightWheel] = (rec_vec[7] << 8) | rec_vec[8];
            angle_curr = ((rec_buf[9] << 8) | rec_buf[10]) * ANGLE_K;
            cout << "Get Pose Succeed " << endl;
        }*/
    }
    //memset(rec_buf, -1, sizeof(rec_buf));
}

void BaseControl::CmdCallback(const geometry_msgs::TwistConstPtr& msg)
{
    cmd_msg = *msg;
}

void BaseControl::SetSpeed()
{
    string SetSpeedCmd;
    float right_wheel_velocity = 0.0;
    float leftWheel_velocity = 0.0;
    char WL_H, WL_L, WR_H, WR_L, SUM;

    /*
    差速推导公式参考：COS495-Lecture5-Odometry.pdf
    right_wheel_velocity = vx + wheels_separation_ * vth * 0.5;
    leftWheel_velocity = vx - wheels_separation_ * vth * 0.5;
    */

    //两个轮子的速度mm/s,放大1000倍，精确到0.001m/s
    right_wheel_velocity = 1000 * (cmd_msg.linear.x + wheels_separation_ * cmd_msg.angular.z * 0.5);
    leftWheel_velocity = 1000 * (cmd_msg.linear.x - wheels_separation_ * cmd_msg.angular.z * 0.5);

    std::cout << "right_speed: "<< right_wheel_velocity << " left_speed: " << leftWheel_velocity << std::endl;

    WL_H = (char)(((int)(leftWheel_velocity) >> 8) & 0xFF);
    WL_L = (char)((int)(leftWheel_velocity) & 0xFF);
    WR_H = (char)(((int)(right_wheel_velocity) >> 8) & 0xFF);
    WR_L = (char)((int)(right_wheel_velocity) & 0xFF);
    SUM = (char)(((WL_H + WL_L + WR_H + WR_L) & 0x3F) + 0x30);

#if 1
    SetSpeedCmd.clear();
    SetSpeedCmd.push_back(0x54);
    SetSpeedCmd.push_back(0x45);
    SetSpeedCmd.push_back(0x07);
    SetSpeedCmd.push_back('S');
    SetSpeedCmd.push_back('S');
    SetSpeedCmd.push_back(WL_H);
    SetSpeedCmd.push_back(WL_L);
    SetSpeedCmd.push_back(WR_H);
    SetSpeedCmd.push_back(WR_L);
    SetSpeedCmd.push_back(SUM);
    SetSpeedCmd.push_back('#');

    
    /*for(int i = 0 ; i < 11; i++)
        printf("0x%x ",SetSpeedCmd[i]);
    printf("\n");
    */
    write(sp, buffer(SetSpeedCmd, SetSpeedCmd.length()));
    //cout << "SetSpeedCmd: " << SetSpeedCmd << "size: " << SetSpeedCmd.size() << endl;

#else
    SetSpeedCmd.clear();
    char stop[] = {0x54, 0x45,  0x07, 'S', 'S', 0x00, 0xc8, 0x00, 0xc8, 0x40, '#'};
    //char stop[] = {0x54, 0x45,  0x07, 'S', 'S', 0x00, 0x00, 0x00, 0x00, 0x30, '#'};
    for(int i = 0; i < sizeof(stop); i++)
        SetSpeedCmd.push_back(stop[i]);
    write(sp, buffer(SetSpeedCmd, SetSpeedCmd.length()));
    //cout << "buffer: " << SetSpeedCmd.length() << endl;
#endif
}

void BaseControl::PublishOdom()
{
    float vx = 0.0;//机器人坐标系
    float vth = 0.0;
    float delta_time = 0.0;
    float delta_th = 0.0;
    float delta_s = 0.0;
    float delta_x = 0.0;
    float delta_y = 0.0;
    float dis[2];//单位m

    /*  
    vs = (right_wheel_velocity + leftWheel_velocity) / 2.0;
    vth = (right_wheel_velocity - leftWheel_velocity) / (2.0 * wheels_separation_);
    */
    current_time = ros::Time::now();
    if(FistTime){
        last_time = current_time;
    }

    delta_time = (current_time - last_time).toSec();

    //计算每个轮子行走的距离
    dis[LeftWheel] = (delta_time * velocity[LeftWheel]) / 1000.0;
    dis[RightWheel] = (delta_time * velocity[RightWheel]) / 1000.0;

    //delta_th = (encoder_dis[RightWheel] - encoder_dis[LeftWheel])/(2 * wheels_separation_);
    delta_th = angle_curr - angle_last;
    delta_s = (dis[LeftWheel] + dis[RightWheel]) * 0.5;

    if(FistTime){
        vx = 0.0;
        vth = 0.0;
        FistTime = false;
    }else{
        vx = delta_s / delta_time;
        vth = delta_th / delta_time;
    }

    //累计坐标
    th_pos += delta_th;
    x_pos += delta_s * cos(th_pos + th_pos/2.0);
    y_pos += delta_s * sin(th_pos + th_pos/2.0);

    //递归
    angle_last = angle_curr;
    last_time = current_time;

    //send TF
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_pos);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    //publish odom
    nav_msgs::Odometry odom;
    //header
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //pose
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;
    //velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;
    //publish the message
    odom.pose.covariance[0]  = 0.1;
    odom.pose.covariance[7]  = 0.1;
    odom.pose.covariance[35] = 0.2;
    odom.pose.covariance[14] = DBL_MAX; // set a non-zero covariance on unused
    odom.pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this
    odom.pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf
    odom_pub.publish(odom);


}

void BaseControl::GetPoseData()
{
    string GetPoseCmd;

    GetPoseCmd.push_back(0x54);
    GetPoseCmd.push_back(0x45);
    GetPoseCmd.push_back(0x02);
    GetPoseCmd.push_back('G');
    GetPoseCmd.push_back('P');
    GetPoseCmd.push_back('#');

    write(sp, buffer(GetPoseCmd, GetPoseCmd.length()));
}

int main(int argc ,char* argv[])
{
  ros::init(argc, argv, "base_control_node");

  BaseControl base_control;

  return 0;
}
