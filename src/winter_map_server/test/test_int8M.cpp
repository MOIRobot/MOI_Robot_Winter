#include "ros/ros.h"  
#include "std_msgs/Float32.h"  
#include <sstream>  
#include <iostream>  
#include "std_msgs/UInt8MultiArray.h"  
//#include "std_msgs/MultiArrayDimension.h"  
using namespace std;  
int main(int argc, char **argv)  
{  
  if(argc != 3){
    std::cout << "Usage [] $1 $2\n";
    return -1;
  }
  
  ros::init(argc, argv, "talker");  
  ros::NodeHandle n;  
  
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8MultiArray>("/change_map", 1);  
  //等待创建链接
  ros::Duration(5).sleep();
  /*ros::Rate loop_rate(1);  
  while (ros::ok())  
  { */ 
    
    //std_msgs::MultiArrayLayout msg;  
    const unsigned int data_sz = 2;  
    std_msgs::UInt8MultiArray m;  
	/*
	m.layout.dim.push_back(std_msgs::MultiArrayDimension());  
	m.layout.dim[0].size = data_sz;  
	m.layout.dim[0].stride = 1;  
	m.layout.dim[0].label = "bla";
	*/  
	cout<<"==========================="<<endl;  
	  
	// only needed if you don't want to use push_back  
	m.data.resize(data_sz);  
	m.data[0] = atoi(argv[1]);  
	m.data[1] = atoi(argv[2]); 
	
    chatter_pub.publish(m); 
    ros::Duration(1).sleep();
    /*
    ros::spinOnce();  
    loop_rate.sleep();  
  }*/
  
  
  return 0;  
} 

