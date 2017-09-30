#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Header.h"
#include <time.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "talkerUltraSound");
  ros::NodeHandle n;
  ros::Publisher ultrasound_pub = n.advertise<sensor_msgs::Range>("UltraSoundPublisher", 10);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    sensor_msgs::Range msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/ultrasound";
    msg.header = header;
    msg.field_of_view = 1;
    msg.min_range = 0;
    msg.max_range = 5;
    msg.range = 1;//rand()%3;

    tf::TransformBroadcaster broadcaster;
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.6,0.00,0.1)),
        ros::Time::now(),"base_link", "ultrasound"));


    ultrasound_pub.publish(msg);

    loop_rate.sleep();
  }
  return 0;
}
