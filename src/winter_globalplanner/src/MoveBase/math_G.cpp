#include <math.h>
#include <winter_globalplanner/math_G.h>
namespace agv
{
double MathG::normalize_angle(double angle)
{
    if( angle > M_PI)
        angle -= 2.0 * M_PI;
    if( angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
double MathG::Slope(double GoalX,double CurrentX,double GoalY,double CurrentY)
{
	if ((GoalX-CurrentX)==0.0)
	{
		if (GoalY>CurrentY)
			{return 10000000.0;}
		else
			{return -10000000.0;}
	}
	else
	{
		return (GoalY-CurrentY)/(GoalX-CurrentX);
	}
}
double MathG::Slope(geometry_msgs::PoseStamped goalpose,geometry_msgs::PoseStamped currentpose)
{
	double GoalX=goalpose.pose.position.x;
	double GoalY=goalpose.pose.position.y;
	double CurrentX=currentpose.pose.position.x;
	double CurrentY=currentpose.pose.position.y;
	
	return Slope(GoalX,CurrentX, GoalY,CurrentY);
}
geometry_msgs::Point MathG::CrossPoint(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2,
																geometry_msgs::PoseStamped pose3,geometry_msgs::PoseStamped pose4)
{
	geometry_msgs::Point crossPoint;
	double x1=pose1.pose.position.x;
	double y1=pose1.pose.position.y;
	double x2=pose2.pose.position.x;
	double y2=pose2.pose.position.y;
	
	double x3=pose3.pose.position.x;
	double y3=pose3.pose.position.y;
	double x4=pose4.pose.position.x;
	double y4=pose4.pose.position.y;
	if (x2==x1)
	{
		double k2=(y4-y3)/(x4-x3);
		double c2=y3-k2*x3;
		double Y=k2*x1+c2;
		 crossPoint.x=x1;
		 crossPoint.y=Y;
		return crossPoint;
	}
	if (x3==x4)
	{
		double k1=(y2-y1)/(x2-x1);
		double c1=y1-k1*x1;
		double Y=k1*x3+c1;
		 crossPoint.x=x3;
		 crossPoint.y=Y;
		return crossPoint;
	}
	double k2=(y4-y3)/(x4-x3);
	double c2=y3-k2*x3;
	double k1=(y2-y1)/(x2-x1);
	double c1=y1-k1*x1;
	double X=(c1-c2)/(k2-k1);
	double Y=(k1*c2-c1*k2)/(k1-k2);
	 crossPoint.x=X;
	 crossPoint.y=Y;
	return crossPoint;
}
void MathG::CrossPoint(geometry_msgs::PoseStamped pose1,geometry_msgs::PoseStamped pose2,
																		 geometry_msgs::PoseStamped pose3,geometry_msgs::PoseStamped pose4,
																		 geometry_msgs::PoseStamped& pose)
{
	double x1=pose1.pose.position.x;
	double y1=pose1.pose.position.y;
	double x2=pose2.pose.position.x;
	double y2=pose2.pose.position.y;
	
	double x3=pose3.pose.position.x;
	double y3=pose3.pose.position.y;
	double x4=pose4.pose.position.x;
	double y4=pose4.pose.position.y;
	if (x2==x1)
	{
		double k2=(y4-y3)/(x4-x3);
		double c2=y3-k2*x3;
		double Y=k2*x1+c2;
		 pose.pose.position.x=x1;
		 pose.pose.position.y=Y;
		return ;
	}
	if (x3==x4)
	{
		double k1=(y2-y1)/(x2-x1);
		double c1=y1-k1*x1;
		double Y=k1*x3+c1;
		 pose.pose.position.x=x3;
		 pose.pose.position.y=Y;
		return;
	}
	double k2=(y4-y3)/(x4-x3);
	double c2=y3-k2*x3;
	double k1=(y2-y1)/(x2-x1);
	double c1=y1-k1*x1;
	double X=(c1-c2)/(k2-k1);
	double Y=(k1*c2-c1*k2)/(k1-k2);
	 pose.pose.position.x=X;
	 pose.pose.position.y=Y;
}																	
double MathG::canculateAngle(double GoalX,double GoalY,double CurrentX,double CurrentY)
{
	double ERR_X=GoalX-CurrentX;
	double ERR_Y=GoalY-CurrentY;
	if (ERR_X>0)
	{
		return atan(ERR_Y/ERR_X);
	}
	else if( ERR_X<0)
	{
		if (ERR_Y>0)
			{return atan(ERR_Y/ERR_X)+M_PI;}
		else
			{return atan(ERR_Y/ERR_X)-M_PI;}
	}
	else
	{
		if (ERR_Y>0)
			return M_PI/2.0;
		else
			return 0-M_PI/2.0;
	}
}
double MathG::canculateAngle(geometry_msgs::PoseStamped goalpose,geometry_msgs::PoseStamped currentpose)
{
	double GoalX=goalpose.pose.position.x;
	double GoalY=goalpose.pose.position.y;
	double CurrentX=currentpose.pose.position.x;
	double CurrentY=currentpose.pose.position.y;
	return canculateAngle( GoalX,GoalY,CurrentX, CurrentY);
}
double MathG::canculateDistance(double GoalX,double GoalY,double CurrentX,double CurrentY)
{
	return sqrt(pow((GoalX-CurrentX),2)+pow((GoalY-CurrentY),2));
}
double MathG::canculateDistance(geometry_msgs::PoseStamped goalpose,geometry_msgs::PoseStamped currentpose)
{
	double GoalX=goalpose.pose.position.x;
	double GoalY=goalpose.pose.position.y;
	double CurrentX=currentpose.pose.position.x;
	double CurrentY=currentpose.pose.position.y;
	return canculateDistance( GoalX, GoalY,CurrentX,CurrentY);
}

double MathG::canculateYaw(const geometry_msgs::PoseStamped &pose)
{
	return tf::getYaw(pose.pose.orientation);
}
double MathG::quat_to_angle(const geometry_msgs::PoseStamped &pose)
{
	return tf::getYaw(pose.pose.orientation);
}
void MathG::getYaw(double& yaw)
{
	 tf::StampedTransform transform;
	try{
		tf_listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
		}catch (tf::TransformException ex){
		try{
			tf_listener.lookupTransform("/map", "/robot_0/base_link",  ros::Time(0), transform);
		}catch(tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	}
	
	 //只获得机器人的角度
	 yaw=tf::getYaw(transform.getRotation());
}
double MathG::getYaw(void)
{
	 tf::StampedTransform transform;
	try{
		tf_listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
		}catch (tf::TransformException ex){
		try{
			tf_listener.lookupTransform("/map", "/robot_0/base_link",  ros::Time(0), transform);
		}catch(tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	}
	//只获得机器人的角度
	 return tf::getYaw(transform.getRotation());
}
void MathG::getGolbalPosition(geometry_msgs::PoseStamped &pose)
{
	 tf::StampedTransform transform;
	try{
		tf_listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
		}catch (tf::TransformException ex){
		try{
			tf_listener.lookupTransform("/map", "/robot_0/base_link",  ros::Time(0), transform);
		}catch(tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	}
	//获得机器人的全局位置
	 pose.pose.position.x=transform.getOrigin().x();
	 pose.pose.position.y=transform.getOrigin().y();
	 pose.pose.position.z=transform.getOrigin().z();
	 pose.pose.orientation.x=transform.getRotation().x();
	 pose.pose.orientation.y=transform.getRotation().y();
	 pose.pose.orientation.z=transform.getRotation().z(); 
	 pose.pose.orientation.w=transform.getRotation().w();
	 
	 //pose.pose.orientation=transform.getRotation();
} 
void MathG::getGolbalPosition( double &X,double &Y,double &yaw)
{
	 tf::StampedTransform transform;
	try{
		tf_listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
		}catch (tf::TransformException ex){
		try{
			tf_listener.lookupTransform("/map", "/robot_0/base_link",  ros::Time(0), transform);
		}catch(tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	}
	//获得机器人的全局位置
	 X=transform.getOrigin().x();
	 Y=transform.getOrigin().y();
	 yaw=tf::getYaw(transform.getRotation());
} 
 void MathG::getGolbalPosition( double &X,double &Y)
{
	 tf::StampedTransform transform;
	try{
		tf_listener.lookupTransform("/map", "/base_link",  ros::Time(0), transform);
		}catch (tf::TransformException ex){
		try{
			tf_listener.lookupTransform("/map", "/robot_0/base_link",  ros::Time(0), transform);
		}catch(tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	}
	//获得机器人的全局位置
	 X=transform.getOrigin().x();
	 Y=transform.getOrigin().y();
} 

}
