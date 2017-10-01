/*
 * 订阅机器人的速度，发布轮子的Joint信息,发布odom消息，发布TF关系
 * 用于模仿差速机器人底盘
 * 2017.9.11 by Bob
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

class PublishOdom
{
private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber winterspeed_sub;
    ros::Time current_time,last_time;
    ros::Publisher joint_pub;
    tf::TransformBroadcaster odom_broadcaster;

    double x_sum;
    double y_sum;
    double th_sum;
    
    float wheels_separation;//轮子的间距
    float wheels_radius;//轮子的半径
    float v[2];
    float p[2];

    bool first_time;

    geometry_msgs::Twist new_cmd_vel;
    
public:
    PublishOdom(){
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
        joint_pub = n.advertise<sensor_msgs::JointState>("wheel_joint_states", 1);
        winterspeed_sub = n.subscribe("cmd_vel", 1, &PublishOdom::cmd_vel_callback, this);

        wheels_radius = 0.075;
        wheels_separation = 0.5;
        x_sum = 0.0;
        y_sum = 0.0;
        th_sum = 0.0;
        first_time = true;
        v[0] = 0.0;
        v[1] = 0.0;
        p[0] = 0.0;
        p[1] = 0.0;

        float vx = 0.0;
        float vth = 0.0;
        double dt = 0.0;
        double delta_s = 0.0;
        double delta_th = 0.0;

        ros::Rate loop_rate(50);

        while(ros::ok())
        {
            
            if(first_time){
                last_time = ros::Time::now();
                current_time = last_time;
                first_time = false;
            }
            current_time=ros::Time::now();

            vx = new_cmd_vel.linear.x;//the x speed of the winter
            vth = new_cmd_vel.angular.z;//the thea speed of the winter

            //compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).toSec();
            delta_s  = vx * dt; 
            delta_th = vth * dt;

            th_sum += delta_th;
            x_sum  += delta_s * cos(th_sum + delta_th / 2.0);
            y_sum  += delta_s * sin(th_sum + delta_th / 2.0);
            
            /*
            差速推导公式参考：COS495-Lecture5-Odometry.pdf
            right_wheel_velocity = vx + wheels_separation_ * vth * 0.5;
            leftWheel_velocity = vx - wheels_separation_ * vth * 0.5;
            */
            v[0] = vx + (wheels_separation) * vth * 0.5;
            v[1] = vx - (wheels_separation) * vth * 0.5;

            p[0] += (v[0] / wheels_radius) * dt;
            p[1] += (v[1] / wheels_radius) * dt;
            
            last_time = current_time;


            sensor_msgs::JointState joint_states;
            //update joint_states
            joint_states.header.stamp = ros::Time::now();
            joint_states.name.resize(2);
            joint_states.position.resize(2);
            joint_states.name[0] ="right_wheel_to_base_link";
            joint_states.position[0] = p[0];
            joint_states.name[1] ="left_wheel_to_base_link";
            joint_states.position[1] = p[1];
            //send the joint state and transform
            joint_pub.publish(joint_states);


            //since all odometry is 6DOF we'll need a quaternion created from yaw
            /* 将里程计的偏航角转换成四元数，四元数效率高，这样使用二维和三维的功能包是一样的.*/
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
            odom.twist.twist.angular.z = vth;

            //publish the message
            odom.pose.covariance[0]  = 0.1;
            odom.pose.covariance[7]  = 0.1;
            odom.pose.covariance[35] = 0.2;

            odom.pose.covariance[14] = DBL_MAX; // set a non-zero covariance on unused
            odom.pose.covariance[21] = DBL_MAX; // dimensions (z, pitch and roll); this
            odom.pose.covariance[28] = DBL_MAX; // is a requirement of robot_pose_ekf
            odom_pub.publish(odom);

            loop_rate.sleep();

            ros::spinOnce();

        }
    }

    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        new_cmd_vel = *cmd_msg;
    }

};
int main(int argc,char** argv)
{
    ros::init(argc,argv,"odometry_publiser");
    // topic you want to publish
    PublishOdom PublishOdomObject;
    
    return 0;
}
