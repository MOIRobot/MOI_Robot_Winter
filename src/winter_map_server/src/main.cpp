/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/UInt8MultiArray.h"//第几层和哪个电梯出来
//#include "std_msgs/Int8.h"//第几层和哪个电梯出来
#include "yaml-cpp/yaml.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res)
    {
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
      unsigned int floor_num;
      unsigned int lift_num;
      unsigned int current_lift = 0;//当前所在的电梯号
      map_changed = false;

      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));

      deprecated = (res != 0);
      //如果没有指定地图的精度，就读取yaml文件
      //if (!deprecated) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("Map_server could not open %s.", fname.c_str());
          exit(-1);
        }

        //解析yaml文件
#ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        
        try { 
          doc["map_num"] >> floor_num; 
          std::cout << "\nfloor_num:" <<floor_num << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a floor_num tag or it is invalid.");
          exit(-1);
        }
        
        try { 
          doc["initpose_num"] >> lift_num; 
          std::cout << "\nlift_num:" <<lift_num << std::endl;
        } catch (YAML::InvalidScalar) { 
          printf("The map does not contain a lift_num tag or it is invalid.");
          exit(-1);
        }
            
        //先读取层数目再定义变量，不然不知道有几层
        std::string mapfname[floor_num];   
        double origin[floor_num][3];
        double init_pose[floor_num][lift_num][3];//电梯口的位置
        
        try { 
          doc["resolution"] >> res; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["default_map"] >> current_floor; 
          printf("\ncurrent_floor: %d", current_floor);
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a default_floor tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["negate"] >> negate; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a negate tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["occupied_thresh"] >> occ_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          doc["free_thresh"] >> free_th; 
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
          exit(-1);
        }
        try { 
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
            ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
            exit(-1);
          }
        } catch (YAML::Exception) { 
          ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
          mode = TRINARY;
        }
        try { 
          for(int i = 0; i < floor_num; i++){
            doc["origin"][i][0] >> origin[i][0]; 
            doc["origin"][i][1] >> origin[i][1]; 
            doc["origin"][i][2] >> origin[i][2]; 
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an origin tag or it is invalid.");
          exit(-1);
        }
        try { 
          for(int i = 0; i < floor_num; i++){
            doc["image"][i] >> mapfname[i]; 
            
            std::cout << "\nmapfname" << i << ":"<<mapfname[i] << std::endl;
            
            // TODO: make this path-handling more robust
            if(mapfname[i].size() == 0)
            {
              ROS_ERROR("The image tag cannot be an empty string.");
              exit(-1);
            }
            if(mapfname[i][0] != '/')
            {
              // dirname can modify what you pass it
              char* fname_copy = strdup(fname.c_str());
              mapfname[i] = std::string(dirname(fname_copy)) + '/' + mapfname[i];
              free(fname_copy);
            }
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an image tag or it is invalid.");
          exit(-1);
        }
        
        try { 
          for(int i = 0; i < floor_num; i++){
			  for(int j = 0;j < lift_num; j++){
				doc["init_pose"][i][j][0] >> init_pose[i][j][0]; 
				doc["init_pose"][i][j][1] >> init_pose[i][j][1]; 
				doc["init_pose"][i][j][2] >> init_pose[i][j][2]; 
		    }
          }
        } catch (YAML::InvalidScalar) { 
          ROS_ERROR("The map does not contain an init_pos tag or it is invalid.");
          exit(-1);
        }
        
      //发布初始位姿
      initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
      
      //监听楼层和电梯号
      floor_sub = n.subscribe("/change_map", 1, &MapServer::floorCallback, this);
      
	   //调用service ，清除原来位置的动态障碍层
	  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	  
	  //静态地图服务
      service = n.advertiseService("static_map", &MapServer::mapCallback, this);
        
	  //根据不同的楼层加载不同的地图，加载默认楼层地图
	  ROS_INFO("Loading map from image \"%s\"", mapfname[current_floor].c_str());
	  
	  //加载地图数据到map_resp_中
	  map_server::loadMapFromFile(&map_resp_,
								mapfname[current_floor].c_str(),
								res,negate,occ_th,free_th, 
								origin[current_floor], mode);

	  //设置地图数据帧的信息
	  map_resp_.map.info.map_load_time = ros::Time::now();
	  map_resp_.map.header.frame_id = frame_id;
	  map_resp_.map.header.stamp = ros::Time::now();
	  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
			  map_resp_.map.info.width,
			  map_resp_.map.info.height,
			  map_resp_.map.info.resolution);
	  meta_data_message_ = map_resp_.map.info;
      
	  // Latched publisher for metadata
	  //\param latch (optional) If true, the last message published on
	  //this topic will be saved and sent to new subscribers when they connect
	  metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	  metadata_pub.publish( meta_data_message_ );
	  
	  // Latched publisher for data
	  //在话题map上发布地图数据
	  map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	  map_pub.publish(map_resp_.map);
	  
	  ros::Duration(1).sleep();
	  send_initpose(init_pose[current_floor][current_lift][0],
					init_pose[current_floor][current_lift][1],
					init_pose[current_floor][current_lift][2]);
	  
      ros::Rate loop_rate(1);
      
      while(ros::ok())
      {
		loop_rate.sleep();
		ros::spinOnce();
		
		if(map_changed)
		{
			std::cout << "\nNew floor map " << std::endl;
			if((floor_msg.data[0] < floor_num) && (floor_msg.data[0] >= 0))
				current_floor = floor_msg.data[0];
			if((floor_msg.data[1] < lift_num) && (floor_msg.data[1] >= 0))
				current_lift = floor_msg.data[1];
				
			std::cout << "\ncurrent_floor: " << current_floor;
			std::cout << "\ncurrent_lift: " << current_lift;
			std::cout << "\ninit_pose[current_floor][current_lift][2]: " 
					  << init_pose[current_floor][current_lift][2]<< "\n";
			map_changed = false;
			
			
			//根据不同的楼层加载不同的地图，加载默认楼层地图
			ROS_INFO("Loading map from image \"%s\"", mapfname[current_floor].c_str());
			
			//清除数据
			//map_resp_.map.data.clear();
			//加载地图数据到map_resp_.map中
			map_server::loadMapFromFile(&map_resp_,
										mapfname[current_floor].c_str(),
										res,negate,occ_th,free_th, 
										origin[current_floor], mode);
			
			//设置地图数据帧的信息
			map_resp_.map.info.map_load_time = ros::Time::now();
			map_resp_.map.header.frame_id = frame_id;
			map_resp_.map.header.stamp = ros::Time::now();
			ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
				  map_resp_.map.info.width,
				  map_resp_.map.info.height,
				  map_resp_.map.info.resolution);
			meta_data_message_ = map_resp_.map.info;
			//重新发布地图
			metadata_pub.publish( meta_data_message_ );
			map_pub.publish( map_resp_.map );
			
			
			//Setting pose
			ros::Duration(1).sleep();
			send_initpose(init_pose[current_floor][current_lift][0],
						  init_pose[current_floor][current_lift][1],
						  init_pose[current_floor][current_lift][2]);
						  
			//Setting pose
			
			//clear costmap
			std_srvs::Empty srv;
			if(client.call(srv)){
				ROS_INFO("\nClear costmap succeed!");
			}else{
			ROS_ERROR("\nFailed to call service /move_base/clear_costmaps");
				//exit(-1);
			}
			//清理之后不能马上进行导航，规划路径会出错
			ros::Duration(2).sleep();
		}// if map_changed
      }//while ok
    }//map_server
    
    
    void send_initpose(double x, double y, double yaw);

  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::Publisher initialpose_pub;
    ros::Subscriber floor_sub;
    ros::ServiceServer service;
    bool deprecated;
    std_msgs::UInt8MultiArray floor_msg;
    //std_msgs::Int8 floor_msg;
    bool map_changed;
    unsigned int current_floor;//当前的楼层
    
    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("Sending map");

      return true;
    }

    void floorCallback(const std_msgs::UInt8MultiArrayConstPtr& msg)
    {
        floor_msg = *msg;
        if(current_floor != floor_msg.data[0])
          map_changed = true;
    }

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

void MapServer::send_initpose(double x, double y, double yaw)
{
	geometry_msgs::PoseWithCovarianceStamped initpose_msg;
	initpose_msg.header.frame_id = "/map";
	initpose_msg.header.stamp = ros::Time::now();
	tf::Quaternion init_pose_quat;
	//geometry_msgs::Quaternion init_pose_quat;
	init_pose_quat = tf::createQuaternionFromYaw(yaw);
	initpose_msg.pose.pose.position.x = x;
	initpose_msg.pose.pose.position.y = y;
	initpose_msg.pose.pose.orientation.x = init_pose_quat.x();
	initpose_msg.pose.pose.orientation.y = init_pose_quat.y();
	initpose_msg.pose.pose.orientation.z = init_pose_quat.z();
	initpose_msg.pose.pose.orientation.w = init_pose_quat.w();
	//注意协方差不能为零，否则会出现TF_NAN_INPUT错误
	initpose_msg.pose.covariance[0] = 0.001;
	initpose_msg.pose.covariance[7] = 0.001;
	initpose_msg.pose.covariance[35] = 0.001;
	initialpose_pub.publish(initpose_msg);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  //默认res=0
  double res = 0;//(argc == 2) ? 0.0 : atof(argv[2]);

  try
  {
    MapServer ms(fname, res);
    //ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

