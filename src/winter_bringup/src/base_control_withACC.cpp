/*
g++ base_control.cpp -lboost_system

通过ROS参数设置：
设备名称：/dev/USB0
波特率：115200

通讯协议：服务机器人通信格式_5.xlsx

改为单片机向上发送编码脉冲数信息。

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
  private_nh.param("loop_rate", loop_rate_, 40);
  private_nh.param("acc_x", acc_x_, 0.5);
  private_nh.param("acc_x", acc_th_, 0.8);
  

  cmd_sub = nh.subscribe("/smooth_cmd_vel", 1, &BaseControl::CmdCallback, this);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);

  infrared_pub = nh.advertise<winter_bringup::Infrared>("/infrared", 1);

  ros::Rate loop_rate(loop_rate_);

  signal(SIGINT, &BaseControl::SigHandler);

  sig_flag = false;

  left_temp = 0; 
  right_temp = 0;

  try{

    InitSerial();

    while(ros::ok())
    {

		
        ParseSerial(); 

        GetPoseData();

        SetSpeed();   
        //执行所有准备好的handler
        size_t handler_num = iosev.poll();///区别poll_one() usr/include/boost/asio/io_service.hpp
        //cout << "handler_num: " << handler_num << endl;                         

        ros::spinOnce();

        loop_rate.sleep();

        if(sig_flag)
        {
            ShutdownRobot();
        }
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

}

void BaseControl::HandleRead(boost::system::error_code ec, std::size_t bytes_transferred)
{
    //cout.write(rec_buf, bytes_transferred);

    bytes_read = bytes_transferred;
    printf("read size: %ld  ", bytes_transferred);
}

void BaseControl::ParseSerial()
{
    boost::asio::async_read_until(sp, rec_buf, '#', boost::bind(&BaseControl::HandleRead, this,  _1, _2));

    //处理接受到的数据
    boost::asio::streambuf::const_buffers_type cbt = rec_buf.data();
    std::string recv_str(boost::asio::buffers_begin(cbt), boost::asio::buffers_end(cbt));
    //str转换为unsigned char*
    //const unsigned char* recv_data = recv_str.c_str();
    size_t recv_size = rec_buf.size();

    std::vector<unsigned char> recv_data(recv_str.c_str(), recv_str.c_str() + recv_str.size() + 1);

    
    printf("rec_buf size: %ld  ", recv_size);
    for(int i = 0 ; i < recv_size; i++)
        printf("%d ",recv_data[i]);
    printf("\n");
    

    rec_buf.consume(recv_size);/// Remove characters from the input sequence.

    printf("rec_buf size: %ld  ", rec_buf.size());
    //Pose Data数据
    if((recv_data[0] == 0x45) && (recv_data[1] == 0x54) && (recv_data[2] == 0x09) && (recv_data[3] == 'G') && (recv_data[4] == 'P'))
    {
        //check sum
        if((((recv_data[5] + recv_data[6] + recv_data[7] + recv_data[8] + recv_data[9] + recv_data[10]) & 0x3F) + 0x30) == recv_data[11])
        {
            encoder_curr[LeftWheel] = (short int)((recv_data[5] << 8) | recv_data[6]);
            encoder_curr[RightWheel] = (short int)((recv_data[7] << 8) | recv_data[8]);
            angle_curr = ((recv_data[9] << 8) | recv_data[10]) * ANGLE_K;
            //std::cout << "left:"<< encoder_curr[LeftWheel] << " right:" << encoder_curr[RightWheel] << " angle_curr: " << angle_curr << std::endl;

            //发布odom，每更新一次数据更新一次odom
            PublishOdom();
        }
    }
    //超声波和红外数据
    else if((recv_data[0] == 0x45) && (recv_data[1] == 0x54) && (recv_data[2] == 0x0C) && (recv_data[3] == 'U') && (recv_data[4] == 'I'))
    {
        std::cout << "ultrasonic" <<std::endl;
        if((((recv_data[5] + recv_data[6] + recv_data[7] + recv_data[8] + recv_data[9] + recv_data[10]+ recv_data[11]+ recv_data[12]) & 0x3F) + 0x30) == recv_data[13])
        {
            //单位：m
            winter_bringup::Infrared infrared_msg;
            infrared_msg.infrared.resize(8);
            infrared_msg.infrared[0] = recv_data[5] / 10.0;
            infrared_msg.infrared[1] = recv_data[6] / 10.0;
            infrared_msg.infrared[2] = recv_data[7] / 10.0;
            infrared_msg.infrared[3] = recv_data[8] / 10.0;
            infrared_msg.infrared[4] = recv_data[9] / 10.0;
            infrared_msg.infrared[5] = recv_data[10] / 10.0;
            infrared_msg.infrared[6] = recv_data[11] / 10.0;
            infrared_msg.infrared[7] = recv_data[12] / 10.0;
            
            infrared_pub.publish(infrared_msg);
	    
            std::cout << recv_data[5] << " "<< recv_data[6] <<" " <<std::endl;
        }
    }
}

void BaseControl::CmdCallback(const geometry_msgs::TwistConstPtr& msg)
{
	
    //cmd_msg = *msg;
    if(msg->linear.x>current_vx_)  
		cmd_msg.linear.x=current_vx_+acc_x_/loop_rate_;
    else
		cmd_msg.linearx=current_vx_-acc_x_/loop_rate_;
	
	 if(msg->angular.z>current_vth_)  
		cmd_msg.angular.z=current_vth_+acc_th_/loop_rate_;
    else
		cmd_msg.angular.z=current_vth_-acc_th_/loop_rate_;
	
}

void BaseControl::ShutdownRobot()
{
    string SetSpeedCmd;
    SetSpeedCmd.push_back(0x54);
    SetSpeedCmd.push_back(0x45);
    SetSpeedCmd.push_back(0x07);
    SetSpeedCmd.push_back('S');
    SetSpeedCmd.push_back('S');
    SetSpeedCmd.push_back(0);
    SetSpeedCmd.push_back(0);
    SetSpeedCmd.push_back(0);
    SetSpeedCmd.push_back(0);
    SetSpeedCmd.push_back(0x30);
    SetSpeedCmd.push_back('#');
    
    write(sp, buffer(SetSpeedCmd, SetSpeedCmd.length()));
    sleep(1.0);

    ros::shutdown();
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

    //std::cout << "right_speed: "<< right_wheel_velocity << " left_speed: " << leftWheel_velocity << std::endl;

    WL_H = (char)(((int)(leftWheel_velocity) >> 8) & 0xFF);
    WL_L = (char)((int)(leftWheel_velocity) & 0xFF);
    WR_H = (char)(((int)(right_wheel_velocity) >> 8) & 0xFF);
    WR_L = (char)((int)(right_wheel_velocity) & 0xFF);
    SUM = (char)(((WL_H + WL_L + WR_H + WR_L) & 0x3F) + 0x30);

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
}

void BaseControl::PublishOdom()
{
    float vs = 0.0;//机器人坐标系
    float vth = 0.0;
    float delta_time = 0.0;
    float delta_th = 0.0;
    float delta_s = 0.0;
    float delta_x = 0.0;
    float delta_y = 0.0;
    float dis[2]={0,0};//单位m
    int encoder_diff[2]={0,0};

    /*  
    vs = (right_wheel_velocity + leftWheel_velocity) / 2.0;
    vth = (right_wheel_velocity - leftWheel_velocity) / (2.0 * wheels_separation_);
    */
    current_time = ros::Time::now();
    if(FistTime){
        last_time = current_time;
        angle_last = angle_curr;
        encoder_last[LeftWheel] = encoder_curr[LeftWheel];
        encoder_last[RightWheel] = encoder_curr[RightWheel];
    }

    //计算差值
    encoder_diff[LeftWheel] = encoder_curr[LeftWheel] - encoder_last[LeftWheel];
    encoder_diff[RightWheel] = encoder_curr[RightWheel] - encoder_last[RightWheel];

    if(encoder_diff[LeftWheel] < -10000)
       encoder_diff[LeftWheel] = encoder_diff[LeftWheel] + 60000;
    else if(encoder_diff[LeftWheel] > 10000)
       encoder_diff[LeftWheel] = encoder_diff[LeftWheel] - 60000;

    if(encoder_diff[RightWheel] < -10000)
        encoder_diff[RightWheel] = encoder_diff[RightWheel] + 60000;
    else if(encoder_diff[RightWheel] > 10000)
        encoder_diff[RightWheel] = encoder_diff[RightWheel] - 60000;

    delta_time = (current_time - last_time).toSec();
    
    //计算每个轮子行走的距离
    dis[LeftWheel] = encoder_diff[LeftWheel] * ODOM_K;     
    dis[RightWheel] = encoder_diff[RightWheel] * ODOM_K;

    left_temp += encoder_diff[LeftWheel];
    right_temp += encoder_diff[RightWheel];

    //std::cout << "left_temp: " << left_temp <<" right_temp:" << right_temp << std::endl;

    delta_th = angle_curr - angle_last;
    delta_s = (dis[LeftWheel] + dis[RightWheel]) / 2.0;

    if(FistTime){
        vs = 0.0;
        current_vx_=vs;
        vth = 0.0;
        current_vth_=vth;
        FistTime = false;
    }else{
        vs = delta_s / delta_time;
        vth = delta_th / delta_time;
        current_vx_=vs;
        current_vth_=vth;
    }

    //累计坐标
    th_pos += delta_th;

    float r_x = delta_s * cos(delta_th / 2.0);
    float r_y = delta_s * sin(delta_th / 2.0);

    delta_x = r_x * cos(th_pos) + r_y * sin(th_pos);
    //delta_y = -1.0 * r_x * sin(th_pos) + r_y * cos(th_pos);
    delta_y = r_x * sin(th_pos) - r_y * cos(th_pos);
    
    //delta_x = delta_s * cos(th_pos + delta_th/2.0);
    //delta_y = delta_s * sin(th_pos + delta_th/2.0);
    x_pos += delta_x;
    y_pos += delta_y;

    //std::cout << "x_pos: " << x_pos << " y_pos:" << y_pos << " th_pos: " << th_pos << std::endl;
    //std::cout << " delta_s: "<< delta_s << std::endl;

    //递归
    angle_last = angle_curr;
    last_time = current_time;
    encoder_last[LeftWheel] = encoder_curr[LeftWheel];
    encoder_last[RightWheel] = encoder_curr[RightWheel];

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
    odom.twist.twist.linear.x = vs;
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

void BaseControl::SigHandler(int sig)
{
  if(sig == SIGINT){
    sig_flag = true;
  }
}

int main(int argc ,char* argv[])
{
  ros::init(argc, argv, "base_control_node");

  BaseControl base_control;

  return 0;
}
