#include <vector>
#include <math.h>
#include <winter_globalplanner/move_base_G.h>
#include <iostream>
using std::cout;
using std::endl;
/*
 * 这份代码是原稿　机器人运动方式是　走一段　加速　匀速　减速　然后停止　然后
 * 开始下一段的路线 2016.12.15
 * 
 * */
namespace agv{
LocalMoveBase::LocalMoveBase(costmap_2d::Costmap2D *costmap,const std::vector<geometry_msgs::Point> &footprint_spec) 
{
	ros::NodeHandle n("~");
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	mPath_pub= n.advertise<nav_msgs::Path>("/mplannerplan", 5);//发布调整后的路径1
	mPath2_pub= n.advertise<nav_msgs::Path>("/mplannerplan2", 5);//发布调整后的路径2
	state_pub=n.advertise<std_msgs::String>("/robot_state", 1);//机器人状态发布
	
	global_path_sub=n.subscribe("/planner/planner/plan", 2,  &LocalMoveBase::global_path_Callback,this);//全局路径订阅者
	laser_scan_sub=n.subscribe("/scan", 20,  &LocalMoveBase::laser_scan_Callback,this);//激光雷达消息订阅者
	
	//全局规划的服务
	gobalpathClient=n.serviceClient<navfn::MakeNavPlan>("/planner/planservice");
	mGlobalGoal_pub=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);//目标发布
	
	bool simulation=false;
	n.param<bool>("simulation",simulation,false);
	if(simulation)
	{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 5);
	}
	else
	{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	}
	n.param<double>("RATE",RATE,20.0);
	n.param<double>("MAX_ANGULAR_Z",MAX_ANGULAR_Z,0.8);
	//最小角速度
	n.param<double>("MIN_ANGULAR_Z",MIN_ANGULAR_Z,0.25);
	//加速度
	n.param<double>("ACC_ANGULAR_Z",ACC_ANGULAR_Z,0.8);
	//直接加速然后减速后的角度值
	MODE1_ANGLE=MAX_ANGULAR_Z*MAX_ANGULAR_Z/ACC_ANGULAR_Z;
	//#直接加速或者减速产生的角度
	MODE2_ANGLE=MODE1_ANGLE/2.0;
	//#角度误差多少范围内接受
	n.param<double>("ANGULAR_Z_ERR",ANGULAR_Z_ERR,0.01);

	//#最大线速度
	n.param<double>("MAX_LINEAR_X",MAX_LINEAR_X,0.4)	;
	//#最小线速度
	n.param<double>("MIN_LINEAR_X",MIN_LINEAR_X,0.05);
	//#最大线速度加速度
	n.param<double>("ACC_LINEAR_X",ACC_LINEAR_X,0.4);
	
	ROS_INFO("MAX_LINEAR_X:%f",MAX_LINEAR_X);
	
	//#直接加速然后减速后的距离
	MODE1_DIS=MAX_LINEAR_X*MAX_LINEAR_X/ACC_LINEAR_X;
	//#直接加速或者减速产生的距离
	MODE2_DIS=MODE1_DIS/2.0;
	//#距离误差多少范围内接受
	n.param<double>("LINEAR_X_ERR",LINEAR_X_ERR,0.05);
	
	NewPath=false;//是否有新的路径出现
	isForwardObstacle=false;//前方是否有障碍物
	isRobotMoving=false;
	
	//创建一个判断机器人轨迹预测会不会碰撞的对象
	mLocalCostmap=new agv::LocalMap_G(costmap,footprint_spec);
	//下面是机器人运动主方程
	movebase();
}
LocalMoveBase::LocalMoveBase(costmap_2d::Costmap2DROS* global_costmap,
															tf::TransformListener* tf,
															const std::vector<geometry_msgs::Point> &footprint_spec) 
{
	
	ros::NodeHandle n("~");
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	mPath_pub= n.advertise<nav_msgs::Path>("/mplannerplan", 5);//发布调整后的路径1
	mPath2_pub= n.advertise<nav_msgs::Path>("/mplannerplan2", 5);//发布调整后的路径2
	state_pub=n.advertise<std_msgs::String>("/robot_state", 1);//机器人状态发布
	
	global_path_sub=n.subscribe("/planner/planner/plan", 2,  &LocalMoveBase::global_path_Callback,this);//全局路径订阅者
	laser_scan_sub=n.subscribe("/scan", 20,  &LocalMoveBase::laser_scan_Callback,this);//激光雷达消息订阅者
	
	//全局规划的服务
	gobalpathClient=n.serviceClient<navfn::MakeNavPlan>("/planner/planservice");
	mGlobalGoal_pub=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);//目标发布
	
	bool simulation=false;
	n.param<bool>("simulation",simulation,false);
	if(simulation)
	{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 5);
	}
	else
	{
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	}
	n.param<double>("RATE",RATE,20.0);
	n.param<double>("MAX_ANGULAR_Z",MAX_ANGULAR_Z,0.8);
	//最小角速度
	n.param<double>("MIN_ANGULAR_Z",MIN_ANGULAR_Z,0.25);
	//加速度
	n.param<double>("ACC_ANGULAR_Z",ACC_ANGULAR_Z,0.8);
	//直接加速然后减速后的角度值
	MODE1_ANGLE=MAX_ANGULAR_Z*MAX_ANGULAR_Z/ACC_ANGULAR_Z;
	//#直接加速或者减速产生的角度
	MODE2_ANGLE=MODE1_ANGLE/2.0;
	//#角度误差多少范围内接受
	n.param<double>("ANGULAR_Z_ERR",ANGULAR_Z_ERR,0.01);

	//#最大线速度
	n.param<double>("MAX_LINEAR_X",MAX_LINEAR_X,0.4)	;
	//#最小线速度
	n.param<double>("MIN_LINEAR_X",MIN_LINEAR_X,0.05);
	//#最大线速度加速度
	n.param<double>("ACC_LINEAR_X",ACC_LINEAR_X,0.4);
	
	ROS_INFO("MAX_LINEAR_X:%f",MAX_LINEAR_X);
	
	//#直接加速然后减速后的距离
	MODE1_DIS=MAX_LINEAR_X*MAX_LINEAR_X/ACC_LINEAR_X;
	//#直接加速或者减速产生的距离
	MODE2_DIS=MODE1_DIS/2.0;
	//#距离误差多少范围内接受
	n.param<double>("LINEAR_X_ERR",LINEAR_X_ERR,0.05);
	
	NewPath=false;//是否有新的路径出现
	isForwardObstacle=false;//前方是否有障碍物
	isRobotMoving=false;
	
	//初始化清除地图的对象　clear_costmap_recovery
	ccr.initialize("my_clear_costmap_recovery", tf, global_costmap);
	
	//初始化可以清除任何层的clear_costmap_recovery_gao 的对象
	n.setParam("my_clear_costmap_recovery_gao/reset_distance", 0.1);//设置清除多少m以外的障碍物
	std::vector<std::string> clearable_layers;
	clearable_layers.push_back( std::string("sonar") );//添加第一层要清除的层 这个名字在movebase costmap参数里配置
	n.setParam("my_clear_costmap_recovery_gao/layer_names", clearable_layers);//在参数空间设置名称
	mapLayerClearer.initialize("my_clear_costmap_recovery_gao", tf, global_costmap,NULL);
	
	//创建一个判断机器人轨迹预测会不会碰撞的对象
	mLocalCostmap=new agv::LocalMap_G(global_costmap->getCostmap(),footprint_spec);
	//下面是机器人运动主方程
	movebase();
}
//申请一条全局路径
bool  LocalMoveBase::getGlobalPath( void)
{
		ROS_INFO("Apply for a new path!");
		int length=MPath[FIANLPATH].poses.size();
		mGlobalGoal_pub.publish(MPath[FIANLPATH].poses[length-1]);
		/*geometry_msgs::PoseStamped pose;
		pose.header.frame_id="/map";
		navfn::MakeNavPlan srv;
		mMath.getGolbalPosition(pose);
		srv.request.start=pose;
		int length=MPath[FIANLPATH].poses.size();
		srv.request.goal=MPath[FIANLPATH].poses[length-1];
		ROS_INFO("begin");
		if( gobalpathClient.call(srv))
		{
			ROS_INFO("Get a new path!");
			//得到path
			//srv.response.path
			return true;
		}
		else
		{
			ROS_INFO("Failed to get a new path!");
			return false;
		}*/
}
void LocalMoveBase::global_path_Callback( const nav_msgs::Path::ConstPtr& path)
{
	try{
	int length=path->poses.size();
	if(length<1)
	{
		ROS_INFO("a valid plan");
		return ;
	}
	/*if(!mLocalCostmap->isLastGoalValid(path->poses[length-1]))
	{
		ROS_INFO("a valid goal");
		return ;
	}*/
	newPathFromAStar(path,MPath[0]);
	NewPath=true;	
	isForwardObstacle=false;
	}
	catch(...)
	{
		ROS_INFO("ERROR");
	}
}
void LocalMoveBase::laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& lasermsg)
{
	
}


void LocalMoveBase::newPathFromAStar( const nav_msgs::Path::ConstPtr& path,nav_msgs::Path &mPath)
{
	int i=15;
	int length;
	double lastGD;//记录上一次的角度
	double GD;//记录当前角度
	int lastj=0;
	
	//清空原来路径的坐标点
	mPath.poses.clear();
	
	//#每１米最少一个目标点　防止长距离角度过小的碰撞
	length=path->poses.size();
	if(length>15)
	{
		lastGD=mMath.quat_to_angle(path->poses[15]);
		mPath.poses.push_back(path->poses[15]);
	}
	while ((i<(length-15)) && (length>15))
	{
		 //当前角度
		GD=mMath.quat_to_angle(path->poses[i]);
		//两次角度差值
		double errDirection=GD-lastGD;
		errDirection=mMath.normalize_angle(errDirection);
		//#0.175 10du 0.35 20 0.524 30degree
		//#遇到拐角的地方　向外进行扩展目标点　根据斜率进行扩展
		if((fabs(errDirection))>0.35)
		{
			//遇到小波动的地方 但是波动角度很大
			//首先计算与上一个点的距离如果很小就可以将这个波动滤掉
			
			double dis=mMath.canculateDistance(path->poses[i],path->poses[i-10]);
			ROS_INFO("dis %f",dis);
			if(dis>0.1)
			{
			//#向外部扩展目标点
			double x=path->poses[i].pose.position.x;
			double y=path->poses[i].pose.position.y;
			double x1=path->poses[i+5].pose.position.x;
			double y1=path->poses[i+5].pose.position.y;
			double x2=path->poses[i-5].pose.position.x;
			double y2=path->poses[i-5].pose.position.y;
			double k1=mMath.Slope(x1,x,y1,y);
			double k2=mMath.Slope(x,x2,y,y2);
			double X=0.0;
			double Y=0.0;
			if (y1>y2)
			{
				if (x1>x2)
				{
					if (k1>k2)
						{
							X=x1;
							Y=y2;
						}
					else
						{
							X=x2;
							Y=y1;
						}
				}
				else
				{
					if (k1>k2)
						{
							X=x2;
							Y=y1;
						}
					else
						{
							X=x1;
							Y=y2;
						}
				}
			}
			else{
				if (x1<x2)
				{
					if (k1>k2)
					{
						X=x1;
						Y=y2;
					}
					else
					{
						X=x2;
						Y=y1;
					}
				}
				else
				{
					if (k1>k2)
						{
							X=x2;
							Y=y1;
						}
					else
						{
							X=x1;
							Y=y2;
						}
					}
			}
			geometry_msgs::PoseStamped mpose;
			mpose.pose.position.x=X;
			mpose.pose.position.y=Y;
			mpose.pose.orientation=path->poses[i].pose.orientation;
			//将扩展点添加到这个录像中
			mPath.poses.push_back(mpose);
			lastGD=GD;
			lastj=i;
		}
		}
		if((i-lastj)>50)
		{
			lastj=i;
			mPath.poses.push_back(path->poses[i]);
		}
		i+=10;
	}
	
	mPath.poses.push_back(path->poses[ length-1]);
	mPath.header.frame_id=path->header.frame_id;
	//最终的路线
	FIANLPATH=0;
}
bool LocalMoveBase::PathFilter(const nav_msgs::Path &mPath,double MinDis,int count)
{
	
	//清空将要使用路径的坐标点
	MPath[count].poses.clear();
	
	MPath[count].header.frame_id=mPath.header.frame_id;
	//传进来路径的长度
	int length=mPath.poses.size();
	//如果只有或者小于三个点
	if (length<4)
	{
		return false;
	}
	//首先将第一个点添加到新的路径中
	MPath[count].poses.push_back(mPath.poses[0]);
	//#从第三个点开始计算
	int i=2;
	int FLAG=0;
	while (i<(length-1))
	{
		double dis=mMath.canculateDistance(mPath.poses[i],mPath.poses[i-1]);
		if(dis<MinDis)
		{
			if ((dis<0.25) and ((fabs(mPath.poses[i-1].pose.position.x-mPath.poses[i].pose.position.x)<0.05) or (fabs(mPath.poses[i-1].pose.position.y-mPath.poses[i].pose.position.y)<0.05)))
			{
					MPath[count].poses.push_back(mPath.poses[i-1]);
					i+=2;
					FLAG=3;
			}
			else
			{
				//#直线角度若相差过小　可能产生尖端
				double angle1=mMath.canculateAngle(mPath.poses[i-2],mPath.poses[i-1]);
				double angle2=mMath.canculateAngle(mPath.poses[i],mPath.poses[i+1]);
				//#30度以内
				if (fabs(mMath.normalize_angle(angle1-angle2))<0.5)
				{
					MPath[count].poses.push_back(mPath.poses[i-1]);
					i+=2;
					FLAG=2;
				}
				else
				{
					geometry_msgs::PoseStamped POSE;
					mMath.CrossPoint(mPath.poses[i-2],mPath.poses[i-1],mPath.poses[i],mPath.poses[i+1],POSE);
					MPath[count].poses.push_back(POSE);
					i+=2;
					FLAG=1;
				}
			}
		}
		else
		{
			MPath[count].poses.push_back(mPath.poses[i-1]);
			i+=1;
			FLAG=2;
		}
	}
	if (i==(length-1))
	{
		MPath[count].poses.push_back(mPath.poses[length-2]);
	}
	MPath[count].poses.push_back(mPath.poses[length-1]);
	//最终的路线
	FIANLPATH=count;
	return true;
}
bool LocalMoveBase::PathFilter_ChooseMainPath(const nav_msgs::Path &mPath,int count)
{
	//清空将要使用路径的坐标点
	MPath[count].poses.clear();
	MPath[count].header.frame_id=mPath.header.frame_id;
	//传进来路径的长度
	int length=mPath.poses.size();
	//长度小于3个不在进行长距离点的选择
	//首先将第一个点添加到新的路径中
	if(length<3)
		return false;
	MPath[count].poses.push_back(mPath.poses[0]);
	double lastAngle=mMath.canculateAngle(mPath.poses[1],mPath.poses[0]);
	for(int i=1;i<(length-1);i++)
	{
		double angle=mMath.canculateAngle(mPath.poses[i+1],mPath.poses[i]);
		double errA=fabs(angle-lastAngle);
		if (errA>6) errA=2*M_PI-errA;
		if (errA>0.1)
		{
			MPath[count].poses.push_back(mPath.poses[i]);
			lastAngle=angle;
		}
			
	}
	MPath[count].poses.push_back(mPath.poses[length-1]);
	//最终的路线
	FIANLPATH=count;
} 
 /*
  * 转到指定的方向上*/
 void LocalMoveBase::rotateToAngle(double goalAngle)
{
	if(NewPath) return;//如果出现了新的路径立即返回
	double current_angle;
	//获取当前的角度
	mMath.getYaw( current_angle);
	//获取两个角度之间的差值
	double turn_angle=mMath.normalize_angle(goalAngle-current_angle);
	//设置旋转加速度  这个值为正的 所以要判断一下旋转方向
	double rotate_acc=ACC_ANGULAR_Z;
	if (turn_angle<0.0)
	{
		rotate_acc=0.0-rotate_acc;
	}
	//记录最初的旋转角
	double O_turn_angle=turn_angle;
	
	geometry_msgs::Twist move_cmd;
	ros::Rate r(RATE);
	double cAngle;
	while ((fabs(turn_angle)>ANGULAR_Z_ERR)  and ros::ok())
	{
		ros::spinOnce();
		cmd_vel_pub.publish(move_cmd);
		r.sleep();
		cAngle=mMath.getYaw();
		turn_angle=mMath.normalize_angle(goalAngle-cAngle);
		bool up=true;
		if (fabs(O_turn_angle)<MODE1_ANGLE)
		{
			//第一种模式 加速然后减速
			if((fabs(turn_angle)>(fabs(O_turn_angle)/2.0)) and not NewPath)	{	up=true;	}
			else {	up=false;}
		}
		else
		{
				//第二种模式 加速 匀速 减速
			if((fabs(turn_angle)>MODE2_ANGLE) and not NewPath)  { up=true;}
			else {up=false;}
		}
		if(up)
		{
			if (fabs(move_cmd.angular.z)<MAX_ANGULAR_Z)
					move_cmd.angular.z+=rotate_acc/RATE;
		}
		else
		{
			if (fabs(move_cmd.angular.z)>MIN_ANGULAR_Z)
					move_cmd.angular.z-=rotate_acc/RATE;
			else
				{
					if(NewPath)
					{
						PublishMoveStopCMD();
						return ;
					}
				}
		}
			
	}
	geometry_msgs::Twist move_stop;
	cmd_vel_pub.publish(move_stop);
	
}
bool LocalMoveBase::moveToGoal_Forward(const geometry_msgs::PoseStamped &currentpose,const geometry_msgs::PoseStamped &goalpose)
{}
bool LocalMoveBase::moveToGoal_Forward(const geometry_msgs::PoseStamped &currentpose,const geometry_msgs::PoseStamped &goalpose,int nextpose)
{
	if(NewPath) return false;//如果出现了新的路径立即返回
	//#获取机器人目标的位置　角度
	if(!mLocalCostmap->isLineValid(currentpose,goalpose))
	{
					ROS_INFO("TISH next PATH is INvalid");
					return false;
	}
	double GX=goalpose.pose.position.x;
	double GY=goalpose.pose.position.y;
	//获取机器人的起始坐标
	double x_start,y_start;
	mMath.getGolbalPosition(x_start,y_start);
	//到目标点的位置
	double goal_distance=mMath.canculateDistance(GX,GY,x_start,y_start);
	
	ROS_INFO("goal_distance:%f",goal_distance);
	
	geometry_msgs::Twist move_cmd;
	
	double distance=0.0;
	double x_current,y_current;
	isRobotMoving=true;
	ros::Rate r(RATE);
	//在运动的过程中实时检测下一个目标点的有效性
	int i=0;
	while (distance<goal_distance and ros::ok())
	{
		//检测3m以内目标点的有效性
		i++;
		if(i%int(RATE)==0)
		{
			if(!mLocalCostmap-> isMetersPathValid(6.0,MPath[FIANLPATH],nextpose))
			{
				isForwardObstacle=true;
				PublishState("MoveErrorZ");
				//申请一条新的路径
				getGlobalPath();
			}
		}
		
		ros::spinOnce();
		cmd_vel_pub.publish(move_cmd);
		r.sleep();
		//获取当前的位置
		mMath.getGolbalPosition(x_current,y_current);
		distance=mMath.canculateDistance(x_current,y_current,x_start,y_start);
		bool up=true;
		if(goal_distance<MODE1_DIS)
		{
			if (!((distance<(goal_distance/2.0)) and not NewPath and not isForwardObstacle)) {up=false;}
		}
		else
		{
			if (!(((goal_distance-distance)>MODE2_DIS)and not NewPath and not isForwardObstacle)) {up=false;}
		}
		if(up)
		{
			if (move_cmd.linear.x<MAX_LINEAR_X) {move_cmd.linear.x+=ACC_LINEAR_X/RATE;}
		}
		else
		{
			if (move_cmd.linear.x>MIN_LINEAR_X)
			{
				move_cmd.linear.x-=ACC_LINEAR_X/RATE;
			}
			else
			{
				if(NewPath)
				{
					//最后让机器人停下来
					PublishMoveStopCMD();
					return false;
				}
				if(isForwardObstacle)
				{
					PublishMoveStopCMD();
					return false;
				}
			}
		}
		}
	geometry_msgs::Twist move_stop_cmd;
	cmd_vel_pub.publish(move_stop_cmd);
	return true;
}
//#传入目标位置　将机器人转动到向该目标点运动的方向
bool LocalMoveBase::rotateToMoveDirection(const geometry_msgs::PoseStamped &currentpose,
																					const geometry_msgs::PoseStamped &goalpose)
{
	//#计算目标点与当前点的全局坐标系下的角度值
	double cx,cy,cAngle;
	mMath.getGolbalPosition(cx,cy,cAngle);
	ROS_INFO("now position x %f  y %f ",cx,cy);
	double goalAngle=mMath.canculateAngle(goalpose.pose.position.x,goalpose.pose.position.y,cx,cy);
	if(!(mLocalCostmap->canRobotRotate(currentpose.pose.position.x,currentpose.pose.position.y,cAngle,goalAngle)))
	{
			ROS_INFO("ERROR! COLLISION WILL Happen");
			PublishState("RotateErrorZ");
			return false;
	}
	
	rotateToAngle(goalAngle);
	return true;
}
//#转动到目标点的方向
bool LocalMoveBase::rotateToGoalDirection(const geometry_msgs::PoseStamped &lastpose,
																					const geometry_msgs::PoseStamped &currentpose)
{
	
	double cAngle=mMath.getYaw();//获取当前机器人方向
	double goalAngle=mMath.quat_to_angle(currentpose);//获取最终方向

	if(!(mLocalCostmap->canRobotRotate(currentpose.pose.position.x,currentpose.pose.position.y,cAngle,goalAngle)))
	{
			ROS_INFO("ERROR! COLLISION WILL Happen");
			PublishState("RotateErrorZ");
			return false;
	}
	rotateToAngle(goalAngle);
	return true;
}
void LocalMoveBase::PublishState(const char* message)
{
	std_msgs::String state;
	state.data=message;
	state_pub.publish(state);
}
void LocalMoveBase::PublishMoveStopCMD(void)
{
	geometry_msgs::Twist move_stop_cmd;
	cmd_vel_pub.publish(move_stop_cmd);
}

//选择最后的运动路线
void LocalMoveBase::ChoosePath(void)
{
	int i=0;
	//传入当前的路径 i+1 为下一条的路径
	if(PathFilter(MPath[i],0.5,i+1))
	{
		i++;//如果成功则移动到下一条线
	}
	if(PathFilter(MPath[i],0.5,i+1))
	{
		i++;//如果成功则移动到下一条线
	}
	if(PathFilter(MPath[i],0.5,i+1))
	{
		i++;//如果成功则移动到下一条线
	}
	
	/*if(PathFilter_ChooseMainPath(MPath[i],i+1))
	{
		i++;
	}*/
	mPath_pub.publish(MPath[i]);
	FIANLPATH=i;
	
}
void LocalMoveBase::movebase(void)
{
	std_msgs::String state;
	bool done=false;
	ros::Rate r(RATE);
	while (ros::ok())
	{
		if(NewPath)
		{
			ChoosePath();//选择路线
			PublishState("StartMoveZ");
			NewPath=false;
			int i=1;
			int length=MPath[FIANLPATH].poses.size();
			ROS_INFO("has %d goals",length-1);
			ccr.runBehavior();
			while (i<length)
			{
				//PublishState("RotateZ");
				ROS_INFO("--------------------->");
				if(i<(length-2))
				{
					//判断下一个目标点是否有效 
					done=mLocalCostmap->isNextGoalValid(MPath[FIANLPATH],i-1);
					if(!done) 
					{
						break;
					}
				}
				done=rotateToMoveDirection(MPath[FIANLPATH].poses[i-1],MPath[FIANLPATH].poses[i]);
				if(!done) 
				{
					ROS_INFO("LAST TIME Prediction ERROR");
					getGlobalPath();
					break;
				}
				done=moveToGoal_Forward(MPath[FIANLPATH].poses[i-1],MPath[FIANLPATH].poses[i],i);
				
				if(!done) 
				{
					break;
				}
				//清空一下周围地图
				ccr.runBehavior();
				i+=1;
				ros::spinOnce();
			}
			if(done)
			{
			if(rotateToGoalDirection(MPath[FIANLPATH].poses[i-2],MPath[FIANLPATH].poses[i-1]))
				{
					PublishState("GoalReachedZ");
					ccr.runBehavior();
					
					//清除超声波障碍物层
					mapLayerClearer.clearOnelayer("sonar",0.1);
				}
				
			}
			else
			{
				
			}
			
		}
		ros::spinOnce();
		r.sleep();
	}
}
}//namespace

