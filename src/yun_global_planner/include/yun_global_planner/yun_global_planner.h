/*
全局规划器的原理：
1、首先加载之前用PointPathManager_3.py工具定义并保存好的两个文件，
保存有线段的lines.txt文件和保存有所有位点的waypoints.txt文；

2、然后用线段数据和位点数据构建出一个地图，然后根据机器人的当前位置和
目标位置在地图上用Dijkstra算法搜索出最短的路径，最后通过贝塞尔曲线优化路径，
并发布出去。 
* 
* 2017-5-29 by Bob
 */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <boost/algorithm/string.hpp>  //for split
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <angles/angles.h>
#include <nav_msgs/Path.h>
#include <string>
#include <iostream>
#include <fstream> //for ifstream
#include <sstream> //for stringstream
#include <vector>
#include <cstdio> // for EOF 
#include <cmath> //for sqrt
#include <stack>
#include <algorithm> //for reverse

using std::string;
using namespace std;
using namespace boost::algorithm;

//贝塞尔曲线的控制点数
#define BEZIER_CONTROL_POINTS		5

//贝塞尔曲线的最小角度15度
#define MIN_ANGLE_BEZIER			0.261799388

//agv的转向半径
#define AGV_TURN_RADIUS			1.2


typedef struct Waypoint{
	int iID;
	float dX;
	float dY;
	//float dA;
}Waypoint;


typedef struct Line{
	//id
	int id;
	//长度
	float length;
	//两个端点
	Waypoint w0; 
	Waypoint w1;
}Line;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace yun_global_planner {

	class YunGlobalPlanner:public nav_core::BaseGlobalPlanner
	{
	public:
		YunGlobalPlanner();
		
		YunGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		
		/** overridden classes from interface nav_core::BaseGlobalPlanner **/
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
		
		//规划路径
		bool makePlan(const geometry_msgs::PoseStamped& start,
					const geometry_msgs::PoseStamped& goal,
					std::vector<geometry_msgs::PoseStamped>& plan);
		
		/*从硬盘中读取点的数据*/
		void readWaypoints();
		
		/*优化路径*/
		int Optimize(float distance);
		
		/*计算点积*/
		float dot2( Waypoint w, Waypoint v);
		
		/*将字符串转化为数字*/
		template<typename Type> Type stringToNum(const string&);
		
		/*计算贝塞尔曲线*/
		Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t);
		
		//读取线段数据
		void readLines();
		
		/*计算两点之间的距离*/
		float Dist(float x1, float y1, float x2, float y2);
		
		/*填充线段*/
		void fillLine(vector<Waypoint> &, vector<int> &);
		
		/*点到线段的最短距离*/
		float DistP2S(geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb);
		
		//计算交叉点积
		float Dot2( float x1, float y1, float x2, float y2);
		
		//计算在所有线段上距离当前位点最近的一个点，并通过指针返回最短的距离和最近点所在线段的id
		Waypoint calClosestPoint(Waypoint w, float* length, int* line_id);
		
		//利用dijkstra算法选择路径
		void dijkstra(const int numOfNode, //结点的数目
					const int startNode, //起始结点
					float **MAP, //地图数据
					float *distance, //距离
					int *preNode); //路径
					
		//初始化地图
		void initMap(vector<Waypoint> waypoint_vector, vector<Line> lines);
		
		//选择最好的路径
		void selectPath(int startNode, int goalNode, int size);
		
	private:
	
		//储存所有的位点
		vector<Waypoint> vPoints;

		//读取线段的容器
		vector<int> pointPairs;
		
		//储存线段信息的容器
		vector<Line> LineVector;
		
		//保存最优路径的位点
		vector<Waypoint> trace;
		
		//waypoints.txt的路径
		const char* pointPath;
		
		//lines.txt的路径
		const char* linePath;
		
		//地图指针
		float **MAP=NULL;
		
		//判断是否被优化了
		bool isOptimize;
		
		//路径
		nav_msgs::Path gui_path;
		
		//节点句柄
		ros::NodeHandle node_handle_;
		
		//发布gui_path
		ros::Publisher path_pub_;
		
	};
};
#endif












