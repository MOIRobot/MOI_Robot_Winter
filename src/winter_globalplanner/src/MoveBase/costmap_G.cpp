#include <math.h>
#include <winter_globalplanner/costmap_G.h>
#include  <geometry_msgs/PoseStamped.h>
namespace agv
{
LocalMap_G::LocalMap_G(costmap_2d::Costmap2D *costmap,const std::vector<geometry_msgs::Point> &footprint_spec)
	{
		 if (costmap != NULL) 
		 {
			world_model_=new base_local_planner::CostmapModel(*costmap);
			//v1.assign(v2.begin(), v2.end());//将v2赋值给v1
			footprint_spec_.assign(footprint_spec.begin(),footprint_spec.end());
			
		}
}
LocalMap_G::~LocalMap_G()
	{
		if (world_model_ != NULL) {
		delete world_model_;
		}
}
bool LocalMap_G::canRobotRotateNextGoal(const geometry_msgs::PoseStamped &nextpose,const geometry_msgs::PoseStamped &goalpose)
{
	double X,Y;
	mMath.getGolbalPosition(X,Y);
	//获得机器人的全局位置
	double start_Angle=mMath.canculateAngle(nextpose.pose.position.x,nextpose.pose.position.y,X,Y);
	double goalAngle=mMath.canculateAngle(goalpose.pose.position.x,goalpose.pose.position.y,nextpose.pose.position.x,nextpose.pose.position.y);
	return canRobotRotate(nextpose.pose.position.x,nextpose.pose.position.y,start_Angle,goalAngle);
}
//机器人位置，footprint，传入机器人的中心坐标 机器人从一个方向转到另一个方向 
bool LocalMap_G::canRobotRotate(double PX, double PY,double start_angle,double end_angle)
{
	if(world_model_==NULL) return false;
	//判断左转还是右转
	double angle=mMath.normalize_angle(end_angle-start_angle);
	int time=int(angle/0.313);//求出次数
	for(int i=0;i<abs(time);i++)
	{
		if(time>=0)
		{
			angle=start_angle+0.313;
		}
		else
		{
			angle=start_angle-0.313;
		}
		double cost=world_model_->footprintCost(PX, PY, angle, footprint_spec_);
		if(cost<0.0) {return false;}
	}
	return true;
}
bool LocalMap_G::canRobotRotate(double PX, double PY)
{
	if(world_model_==NULL) return false;
	for(double angle=0.0;angle<3.1415;angle+=0.313)
	{
		double cost=world_model_->footprintCost(PX, PY, angle, footprint_spec_);
		if(cost<0.0)
			return false;
	}
	for(double angle=0.0;angle>-3.1415;angle-=0.313)
	{
		double cost=world_model_->footprintCost(PX, PY, angle, footprint_spec_);
		if(cost<0.0)
			return false;
	}
	return true;
}
bool LocalMap_G::canRobotRotate(const geometry_msgs::PoseStamped &pose)
{
	double PX=pose.pose.position.x;
	double PY=pose.pose.position.y;
	return canRobotRotate(PX,PY);
}
bool LocalMap_G::canRobotRotateTOGoal(const geometry_msgs::PoseStamped &pose)
{
	//当前点是否可以向下个目标点转向
	double cx,cy,cAngle;
	mMath.getGolbalPosition(cx,cy,cAngle);
	double goalAngle=mMath.canculateAngle(pose.pose.position.x,pose.pose.position.y,cx,cy);
	bool result=canRobotRotate(cx,cy,cAngle,goalAngle);
	return result;
}
//两个点之间的路线是否有效
bool LocalMap_G::isLineValid(const geometry_msgs::PoseStamped &CurrentPose,const geometry_msgs::PoseStamped  &GoalPose)
{
	//计算机器人朝向
	double angle=mMath.canculateAngle(GoalPose,CurrentPose);
	
    double distance=mMath.canculateDistance(GoalPose,CurrentPose);
    
    double x_dis=GoalPose.pose.position.x-CurrentPose.pose.position.x;
    double y_dis=GoalPose.pose.position.y-CurrentPose.pose.position.y;
    //每0.2米检测一下机器人是否会发生碰撞
    int time=(distance/0.2);//一共检测的次数
    for(int i=1;i<=time;i++)
    {
		double PX=CurrentPose.pose.position.x+ x_dis/time;
		double PY=CurrentPose.pose.position.y+ y_dis/time;
		double cost=world_model_->footprintCost(PX, PY, angle, footprint_spec_);
		if(cost<0.0) {return false;}
	}
	return true;
}
//两个点之间的路线是否有效
bool LocalMap_G::isLineValid(double gx,double gy,double cx,double cy)
{
	//计算机器人朝向
	double angle=mMath.canculateAngle(gx,gy,cx,cy);
	
    double distance=mMath.canculateDistance(gx,gy,cx,cy);
    
    double x_dis=gx-cx;
    double y_dis=gy-cy;
    //每0.2米检测一下机器人是否会发生碰撞
    int time=(distance/0.2);//一共检测的次数
    for(int i=1;i<=time;i++)
    {
		double PX=cx+ x_dis/time;
		double PY=cy+ y_dis/time;
		double cost=world_model_->footprintCost(PX, PY, angle, footprint_spec_);
		if(cost<0.0) {return false;}
	}
	return true;
}
bool LocalMap_G::isNextGoalValid(nav_msgs::Path &path,int currentPose)
{
	
	//下一个目标点进行判断
	//判断现在能不能向下一步目标点运动
	//判断能不能转向目标点
	//判断下个目标点处　能不能向下下个目标点转向
	bool done=canRobotRotateTOGoal(path.poses[currentPose+1]);
	if(!done)	{ROS_INFO("NEXT ROTATE IS INVALID");}
	bool done1=isLineValid(path.poses[currentPose],path.poses[currentPose+1]);
	if(!done1)	{ROS_INFO("NEXT PATH IS  INVALID PATH");}
	bool done2=canRobotRotateNextGoal(path.poses[currentPose+1],path.poses[currentPose+2]);
	if(!done2) {ROS_INFO("Robot CANNOT ROTATE NEXT GOAL");}
	if(!(done&&done1&&done2))
	{
		//如果不行　则重新选择下一个目标点
			int i=1;
			geometry_msgs::PoseStamped mpose;
			double X=path.poses[currentPose+1].pose.position.x;
			double Y=path.poses[currentPose+1].pose.position.y;
			double MAX_DIS=mMath.canculateDistance(path.poses[currentPose+2],path.poses[currentPose]);
			while (true)
			{
				double j=0;
				while(j<3.1415*2)
				{
					mpose.pose.position.x=X+i*0.25*cos(j);
					mpose.pose.position.y=Y+i*0.25*sin(j);
					bool result=isLineValid(path.poses[currentPose],mpose);
					if(result)
					{
						//下下目标点处是否能转向
						result=canRobotRotateNextGoal(mpose,path.poses[currentPose+2]);
						if(result)
						{
							//当前点是否可以向下个目标点转向
							result=canRobotRotateTOGoal(path.poses[currentPose+1]);
							if(result)
							{
								ROS_INFO("ORIGINAL NEXT GOAL x %f y%f",path.poses[currentPose+1].pose.position.x,path.poses[currentPose+1].pose.position.y);
								path.poses[currentPose+1].pose.position.x=mpose.pose.position.x;
								path.poses[currentPose+1].pose.position.y=mpose.pose.position.y;
								ROS_INFO("Found a VALID next goal x %f Y %f",path.poses[currentPose+1].pose.position.x,path.poses[   currentPose+1].pose.position.y);
								return true;
							}
						}
					}
					j+=0.214;
				}
			i++;
			if((i*0.15)>MAX_DIS)
			{
				ROS_INFO("CANNOT Found a VALID next goal");
				return false;
			}
		}
	return true;
	}
	else
	{
		return true;
	}
}
bool 	LocalMap_G::choose_aValid_NextGoal(nav_msgs::Path &path,int currentPose)
{
			//如果不行　则重新选择下一个目标点
			int i=1;
			geometry_msgs::PoseStamped mpose;
			double X=path.poses[currentPose+1].pose.position.x;
			double Y=path.poses[currentPose+1].pose.position.y;
			double MAX_DIS=mMath.canculateDistance(path.poses[currentPose+2],path.poses[currentPose]);
			while (true)
			{
				double j=0;
				while(j<3.1415*2)
				{
					mpose.pose.position.x=X+i*0.25*cos(j);
					mpose.pose.position.y=Y+i*0.25*sin(j);
					bool result=isLineValid(path.poses[currentPose],mpose);
					if(result)
					{
						//下下目标点处是否能转向
						result=canRobotRotateNextGoal(mpose,path.poses[currentPose+2]);
						if(result)
						{
							//当前点是否可以向下个目标点转向
							result=canRobotRotateTOGoal(path.poses[currentPose+1]);
							if(result)
							{
								ROS_INFO("ORIGINAL NEXT GOAL x %f y%f",path.poses[currentPose+1].pose.position.x,path.poses[currentPose+1].pose.position.y);
								path.poses[currentPose+1].pose.position.x=mpose.pose.position.x;
								path.poses[currentPose+1].pose.position.y=mpose.pose.position.y;
								ROS_INFO("Found a VALID next goal x %f Y %f",path.poses[currentPose+1].pose.position.x,path.poses[   currentPose+1].pose.position.y);
								return true;
							}
						}
					}
					j+=0.214;
				}
			i++;
			if((i*0.15)>MAX_DIS)
			{
				ROS_INFO("CANNOT Found a VALID next goal");
				return false;
			}
		}
	return true;
	
}
//判断几米以内的路线是否有效
bool LocalMap_G::isMetersPathValid(double meter,nav_msgs::Path &path,int nextPose)
{
	double dis_sum;
	double dis;
	double cx,cy,yaw;
	int length=path.poses.size();
	//获取当前坐标　角度
	mMath.getGolbalPosition( cx,cy,yaw);
	//获取总的米数之间存在的点
	int i=nextPose;
	dis_sum=mMath.canculateDistance(path.poses[nextPose].pose.position.x,path.poses[nextPose].pose.position.y,cx,cy);
	while (dis_sum<meter)
	{
		if(i==(length-1))
		{
			break;
		}
		dis_sum+=mMath.canculateDistance(path.poses[i+1],path.poses[i]);
		i++;
	}
	if(!isLineValid(path.poses[nextPose].pose.position.x,path.poses[nextPose].pose.position.y,cx,cy))
	{
		ROS_INFO("Obstacle!!");
		return false;
	}
	for(int j=nextPose;j<(i-1);j++)
	{
		//机器人与下一个目标点之间的连线是否有效
		bool done1=isLineValid(path.poses[j],path.poses[j+1]);
		if(!done1)	{
			ROS_INFO("Obstacle!!");
			return false;
			}
	}
	return true;
}
bool LocalMap_G::isFinalGoalValid(const geometry_msgs::PoseStamped &LastPose)
{
	double angle=tf::getYaw(LastPose.pose.orientation);
	double cost=world_model_->footprintCost(LastPose.pose.position.x, LastPose.pose.position.y, angle, footprint_spec_);
	if(cost<0.0)
		return false;
	return true;
}							

}
