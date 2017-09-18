#ifndef MATH_G_H_
#define MATH_G_H_

#include <math.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
namespace agv{
	class MathG
	{
	public:
		//MathG(void);
		//计算两个角度之间的差角 时用的
		double normalize_angle(double angle);
		//求两个点的斜率
		/*
		 * 传入参数 目标点X   当前点 X 目标点Y 当前点Y */
		static double Slope(double GoalX,double CurrentX,double GoalY,double CurrentY);
		/*
		 * 求斜率 目标点  当前点*/
		double Slope(geometry_msgs::PoseStamped goalpose,geometry_msgs::PoseStamped currentpose);
		/*
		 * 两条直线的交点*/
		 geometry_msgs::Point CrossPoint(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2,
																		 geometry_msgs::PoseStamped pose3,geometry_msgs::PoseStamped pose4);
		 void CrossPoint(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2,
																		 geometry_msgs::PoseStamped pose3,geometry_msgs::PoseStamped pose4,
																		 geometry_msgs::PoseStamped& pose);
		/*
		 * 计算目标点与当前点的矢量方向*/
		double canculateAngle(double GoalX,double GoalY,double CurrentX,double CurrentY);
		double canculateAngle(geometry_msgs::PoseStamped goalpose,geometry_msgs::PoseStamped currentpose);
		/*
		 * 计算两个点中间的距离*/
		double canculateDistance(double GoalX,double GoalY,double CurrentX,double CurrentY);
		double canculateDistance(geometry_msgs::PoseStamped goalpose,geometry_msgs::PoseStamped currentpose);
		//计算pose的yaw角度
		double canculateYaw(const geometry_msgs::PoseStamped &pose);
		double quat_to_angle(const geometry_msgs::PoseStamped &pose);
		
		//计算当前的朝向　返回角度　
		void getYaw(double& yaw);
		double getYaw(void);
		
		//获取当前的全局位置
		void getGolbalPosition(geometry_msgs::PoseStamped &pose);
		void getGolbalPosition( double &X,double &Y,double &yaw);
		void getGolbalPosition( double &X,double &Y);
		private:
		tf::TransformListener tf_listener;
	};
}

#endif
