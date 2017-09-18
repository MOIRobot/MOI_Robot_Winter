#ifndef COSTMAP_G_H_
#define COSTMAP_G_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <winter_globalplanner/math_G.h>
namespace agv
{
	class LocalMap_G
	{
		public:
				LocalMap_G(costmap_2d::Costmap2D *costmap,const std::vector<geometry_msgs::Point> &footprint_spec);
				~LocalMap_G();
				//机器人是否可以原地旋转
				bool canRobotRotate(double PX, double PY);
				bool canRobotRotate(const geometry_msgs::PoseStamped &pose);
				//机器人是否可以从指定的角度转到指定的角度
				bool canRobotRotate(double PX, double PY,double start_angle,double end_angle);
				bool canRobotRotateTOGoal(const geometry_msgs::PoseStamped &pose);
				//机器人在当前点队下一个点的预测是否有效
				bool canRobotRotateNextGoal(const geometry_msgs::PoseStamped &nextpose,const geometry_msgs::PoseStamped &goalpose);
				//从一个点到下一个点之间的路程是否有效
				bool isLineValid(const geometry_msgs::PoseStamped &CurrentPose,const geometry_msgs::PoseStamped  &GoalPose);
				bool isLineValid(double gx,double gy,double cx,double cy);
				
				//下一个目标点是否有效
				bool isNextGoalValid(nav_msgs::Path &path,int currentPose);
				bool choose_aValid_NextGoal(nav_msgs::Path &path,int currentPose);
				bool isMetersPathValid(double meter,nav_msgs::Path &path,int currentPose);
				//最后一个目标点是否有效
				bool isFinalGoalValid(const geometry_msgs::PoseStamped &LastPose);
				
				void generateValidPath();		
		private:
				base_local_planner::CostmapModel* world_model_;
				std::vector<geometry_msgs::Point> footprint_spec_;
				agv::MathG mMath;
				tf::TransformListener tf_listener;
	};
}


#endif
