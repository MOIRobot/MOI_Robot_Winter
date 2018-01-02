#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int8.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Publisher pub;

  try
  {
    std_msgs::Int8 msg;
    msg.data = 10;
    pub = n.advertise<std_msgs::Int8>("test", 1, true);
    
    ros::Rate loop_rate(10);
    
    //while(ros::ok()){
      pub.publish(msg);
      //loop_rate.sleep();
      //ros::spinOnce();
    //}
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
