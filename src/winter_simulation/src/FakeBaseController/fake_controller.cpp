#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

/*

接收cmdvel速度信息 并按照该速度定时在odom话题上发布里程数据

*/
class OdomPublish{
private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;//
    ros::Publisher joint_pub;//
    ros::Subscriber carspeed_sub;//
    ros::Time current_time,last_time;
    //odom tf transform form base_link to odom
    tf::TransformBroadcaster odom_broadcaster;

    //the speed of the car
    double vx;
    double vy;
    double vth;

    // the positoin of the car || the odometry message of the car
    double x_sum;
    double y_sum;
    double th_sum;
public:
    OdomPublish(){
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
        joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
        carspeed_sub = n.subscribe("cmd_vel", 10, &OdomPublish::cmdvel_callback, this);
        PublishOdom();
    }
    void cmdvel_callback(const geometry_msgs::Twist& input){
            vx=input.linear.x;
            vy=input.linear.y;
            vth=input.angular.z;
            ROS_INFO("I have got a cmd_vel message");
            ROS_INFO("vx is %f",vx);
            ROS_INFO("vy is %f",vy);
            ROS_INFO("vth is %f",vth);

    }
    void PublishOdom()
    {
        ros::Rate loop_rate(20);
         while(ros::ok())
        {
           // ROS_INFO("Publishing");
            current_time=ros::Time::now();
           //compute odometry in a typical way given the velocities of the robot
            double dt = (current_time - last_time).toSec();
            double delta_x = (vx * cos(th_sum) - vy * sin(th_sum)) * dt;
            double delta_y = (vx * sin(th_sum) + vy * cos(th_sum)) * dt;
            double delta_th = vth * dt;
            x_sum += delta_x;
            y_sum += delta_y;
            th_sum += delta_th;

            sensor_msgs::JointState joint_state;
            //update joint_state
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(4);
            joint_state.position.resize(4);
            joint_state.name[0] ="base_to_wheel1";
            joint_state.position[0] = 0;
            joint_state.name[1] ="base_to_wheel2";
            joint_state.position[1] = 0;
            joint_state.name[2] ="base_to_wheel3";
            joint_state.position[2] = 0;
            joint_state.name[3] ="base_to_wheel4";
            joint_state.position[3] = 0;
            //send the joint state and transform
            joint_pub.publish(joint_state);


            //since all odometry is 6DOF we'll need a quaternion created from yaw
            /* 将里程计的偏航角转换成四元数，四元数效率高，这样使用二维和三维的功能包是一样的。*/
           geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_sum);
           //first, we'll publish the transform over tf
           geometry_msgs::TransformStamped odom_trans;
           odom_trans.header.stamp = current_time;
           odom_trans.header.frame_id = "odom";
           odom_trans.child_frame_id = "base_link";
           odom_trans.transform.translation.x = x_sum;
           odom_trans.transform.translation.y = y_sum;
           odom_trans.transform.translation.z = 0.0;
           odom_trans.transform.rotation = odom_quat;
           //send the transform
           odom_broadcaster.sendTransform(odom_trans);


            //next, we'll publish the odometry message over ROS
           nav_msgs::Odometry odom;
           odom.header.stamp = current_time;
           odom.header.frame_id = "odom";
           //set the position
           odom.pose.pose.position.x = x_sum;
           odom.pose.pose.position.y = y_sum;
           odom.pose.pose.position.z = 0.0;
           odom.pose.pose.orientation = odom_quat;
           //set the velocity
           odom.child_frame_id = "base_link";
           odom.twist.twist.linear.x = vx;
           odom.twist.twist.linear.y = vy;
           odom.twist.twist.angular.z = vth;
           //publish the message
           odom_pub.publish(odom);

           last_time = current_time;
           ros::spinOnce();// Handle ROS events
            loop_rate.sleep();
        }

    }
};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"odometry_and_joint_publiser");
    // topic you want to publish
    OdomPublish odom;
    ros::spin();
    return 0;
}
