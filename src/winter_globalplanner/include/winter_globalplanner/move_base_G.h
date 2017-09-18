#ifndef MOVE_BASE_G_H_
#define MOVE_BASE_G_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <winter_globalplanner/math_G.h>
#include <winter_globalplanner/costmap_G.h>
#include <winter_globalplanner/clear_costmap_recovery.h>
#include <navfn/MakeNavPlan.h>
#include <pthread.h>
namespace agv
{
		class LocalMoveBase
		{
			public:
			LocalMoveBase(costmap_2d::Costmap2D *costmap,const std::vector<geometry_msgs::Point> &footprint_spec);
			LocalMoveBase(costmap_2d::Costmap2DROS* global_costmap,tf::TransformListener* tf,const std::vector<geometry_msgs::Point> &footprint_spec);
			//~LocalMoveBase();
			
			//各个订阅者的回调函数
			void global_path_Callback( const nav_msgs::Path::ConstPtr& path);
			void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& lasermsg);
	
			void movebase(void);
			
			//向前进方向运动
			bool moveToGoal_Forward(const geometry_msgs::PoseStamped &pose);
			bool moveToGoal_Forward(const geometry_msgs::PoseStamped &currentpose,const geometry_msgs::PoseStamped &goalpose);
			bool moveToGoal_Forward(const geometry_msgs::PoseStamped &currentpose,const geometry_msgs::PoseStamped &goalpose,int nextpose);
			  /* 转到指定的方向上*/
			void rotateToAngle(double goalAngle);
			//#传入目标位置　将机器人转动到向该目标点运动的方向
			bool rotateToMoveDirection(const geometry_msgs::PoseStamped &currentpose,const geometry_msgs::PoseStamped &goalpose);
			//#转动到目标点的方向
			bool rotateToGoalDirection(const geometry_msgs::PoseStamped &pose);
			bool rotateToGoalDirection(const geometry_msgs::PoseStamped &lastpose,const geometry_msgs::PoseStamped &currentpose);
			//void getGlobalPose(tf::Stamped<tf::Pose>)
			
			//发布状态　用来控制语音
			void PublishState(const char* message);
			//发布停止命令
			void PublishMoveStopCMD(void);
			
			bool getGlobalPath( void);
			//两次对原始路径的滤波
			void newPathFromAStar( const nav_msgs::Path::ConstPtr& path,nav_msgs::Path &mPath);
			//#再次滤波　主要是对path中的拐角进行再次融合变成直角　将小距离的目标点变成大距离的目标点 传入参数 小距离dis 第几次滤波
			bool PathFilter(const nav_msgs::Path &mPath,double MinDis,int count);
			//将短距离的角度相差不大的点 变成一个点
			bool PathFilter_ChooseMainPath(const nav_msgs::Path &mPath,int count);
			void ChoosePath(void);
		private:
			double RATE;//计算速度频率
			double MAX_ANGULAR_Z;//最大角速度
			double MIN_ANGULAR_Z;//#最小角速度
			double ACC_ANGULAR_Z;//#加速度
			double MODE1_ANGLE;//#直接加速然后减速后的角度值 MODE1_ANGLE=MAX_ANGULAR_Z*MAX_ANGULAR_Z/ACC_ANGULAR_Z
			double MODE2_ANGLE;//#直接加速或者减速产生的角度 MODE2_ANGLE=MODE1_ANGLE/2.0
			double ANGULAR_Z_ERR;//#角度误差多少范围内接受
			
			double MAX_LINEAR_X;//X最大线速度
			double MIN_LINEAR_X;//X#最小线速度
			double ACC_LINEAR_X;//#最大线速度加速度
			double MODE1_DIS;//#直接加速然后减速后的距离 MODE1_DIS=MAX_LINEAR_X*MAX_LINEAR_X/ACC_LINEAR_X
			double MODE2_DIS;//#直接加速或者减速产生的距离   MODE2_DIS=MODE1_DIS/2.0
			double LINEAR_X_ERR;//#距离误差多少范围内接受
			
			nav_msgs::Path MPath[5];//最多滤波六次
			int FIANLPATH;
			//nav_msgs::Path mPath1;//一次滤波之后的路径
			//nav_msgs::Path mPath2;//二次滤波之后的路径
			
			//初始化一个数学运算对象
			MathG mMath;
			
			//创建一个判断机器人轨迹预测会不会碰撞的对象
			agv::LocalMap_G* mLocalCostmap;
			
			//易变的值 告诉编译器不要优化这个值
			bool NewPath;//是否有新的路径出现
			bool isForwardObstacle;
			bool isRobotMoving;
			ros::Publisher cmd_vel_pub;//发布速度消息
			
			ros::Publisher mPath_pub;//发布调整后的路径1
			ros::Publisher mPath2_pub;//发布调整后的路径2
			
			ros::Publisher mGlobalGoal_pub;//全局目标点发布是
			
			ros::Publisher state_pub;//机器人状态调整
			
			ros::Subscriber global_path_sub;//全局路径订阅者
			ros::Subscriber laser_scan_sub;//激光雷达消息订阅者
			
			tf::TransformListener tf_listener;
			ros::Rate *loopRate;
			//锁
			ros::ServiceClient gobalpathClient;
			//创建一个清除地图的对象
			MClearCostmapRecovery ccr;
		};
}

 
#endif
