/*增减了描绘优化后的路径的程序
 * 2016.12.6 by Bob
 * 加最后的运动矫正
 * 激光避开障碍
 */
#include <string.h>//
#include <vector>//容器
#include <queue>//队列
#include <stdint.h>//stdint.h是c99中引进的一个标准C库的头文件.定义了int8_t，int16_t， int32_t
#include <ros/ros.h>//stdlib.h中的function
#include <math.h>
#include <cstdlib>//stdlib.h中的函数，比如随机函数
#include <purepursuit_planner/Component.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//诊断信息
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
//actionlib服务器
#include <actionlib/server/simple_action_server.h>
#include <planner_msgs/GoToAction.h>
#include <planner_msgs/goal.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>//导航的路径
#include <sensor_msgs/LaserScan.h>


//没有接受到odom最大时间
#define ODOM_TIMEOUT_ERROR			0.2
//没有接受到map 转换的最大时间
#define MAP_TIMEOUT_ERROR			0.2
//agv的转向半径
#define AGVS_TURN_RADIUS			0.50
//贝塞尔曲线的最小角度15度
#define MIN_ANGLE_BEZIER			0.261799388
//贝塞尔曲线的控制点数
#define BEZIER_CONTROL_POINTS		5
//最小和最大的速度
#define D_LOOKAHEAD_MIN				0.3
#define D_LOOKAHEAD_MAX				1.1
//机器人的中心到轮子的距离
#define D_WHEEL_ROBOT_CENTER    	0.478

//最大速度的级别
#define MAX_SPEED_LVL1				0.5
#define MAX_SPEED_LVL2				0.3
#define MAX_SPEED					0.6
#define AGVS_TURN_SPEED			 0.8//最大的角速度
//距离最后一个位点的最小的距离
#define WAYPOINT_POP_DISTANCE_M		0.05

//当机器人在距离目标0.5米时开始第一次减速，减速时的最大速度为0.15m/s
#define AGVS_FIRST_DECELERATION_DISTANCE 	0.5 
#define AGVS_FIRST_DECELERATION_MAXSPEED	0.20
//当机器人在距离目标前0.5米时开始第二次减速，减速时的最大速度为0.1m/s
#define AGVS_SECOND_DECELERATION_DISTANCE	0.25 
#define AGVS_SECOND_DECELERATION_MAXSPEED	0.10 
#define AGVS_DEFAULT_KR					0.20
//机器人的理论角速度和线速度之间的系数关系
#define AGVS_SPEED_K 						1.0


//枚举类型
enum{
	ODOM_SOURCE = 1,
	MAP_SOURCE  = 2
};

using namespace std;

//! Data structure for a Waypoint
typedef struct Waypoint{
    //! Id of the magnet
    int iID;
    //! X position
    double dX;
    //! Y position
    double dY;
    //! Orientation
    double dA;
    //! Speed to arrive to the point
    double dSpeed;
}Waypoint;

//! class to manage the waypoints current path
//管理当前路径上的位点类
class Path{

public:

	//当前的位点id
	int iCurrentWaypoint;

private:
	//! Mutex to control the access
	//进程锁，用来控制进程
	pthread_mutex_t mutexPath;
	
	//! Vector to store all the Waypoints
	//用来储存位点的容器
	vector <Waypoint> vPoints;
	
	//! Flag to control the optimization
	bool bOptimized;

public:

	//! public constructor
	Path(){
		iCurrentWaypoint = -1;
		//初始化进程锁
		pthread_mutex_init(&mutexPath, NULL);//Initialization for WaypointRoutes' mutex
		bOptimized = false;
	}

	//! Destructor
	~Path(){
		pthread_mutex_destroy(&mutexPath);
	}

	//! Adds a new waypoint
	//添加一个新的位点
	ReturnValue AddWaypoint(Waypoint point){
		//辅助位点
		Waypoint aux;
		//启动进程锁
		pthread_mutex_lock(&mutexPath);
			if(vPoints.size() > 0){	//如果位点容器不为空
				aux = vPoints.back();//得到容器中的最后一个位点
				// Only adds the waypoint if it's different from waypoint before
				//判断要添加的位点和当前位点是否相同
				if( (aux.dX != point.dX) || (aux.dY != point.dY) )
					vPoints.push_back(point);//插入位点
			}else { // First point//第一个位点
				if(iCurrentWaypoint < 0){ //First point
					iCurrentWaypoint = 0;
				}
				vPoints.push_back(point);
			}
		//解开进程锁
		pthread_mutex_unlock(&mutexPath);

		return OK;
	}

	//! Adds a vector of waypoints
	//添加一个容器的位点
	ReturnValue AddWaypoint(vector <Waypoint> po){
		pthread_mutex_lock(&mutexPath);
			//如果是第一个位点
			if(iCurrentWaypoint < 0){ //First point
				iCurrentWaypoint = 0;
			}
			//依次添加容器中所有的位点
			for(int i = 0; i < po.size(); i++){
				vPoints.push_back(po[i]);
			}
		pthread_mutex_unlock(&mutexPath);
	}

	//! Clears the waypoints and magnets
	//清楚所有的位点和磁铁
	void Clear(){
		pthread_mutex_lock(&mutexPath);
			iCurrentWaypoint = -1;
			bOptimized = false;
			vPoints.clear();//清空元素
		pthread_mutex_unlock(&mutexPath);
	}

	//! Returns the size of the vector points
	//返回位点容器的大小
	unsigned int Size(){
		return vPoints.size();
	}

	//! Returns the next waypoint
	//得到下一个位点
	ReturnValue GetNextWaypoint(Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			//如果当前位点id大于0，并且它不是最后一个
			if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < (vPoints.size() - 1)) ){
				*wp = vPoints[iCurrentWaypoint + 1];//得到下一个位点
				ret = OK;
			}

		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Returns the last waypoint
	//返回最后一个位点
	ReturnValue BackWaypoint(Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( vPoints.size() > 0){
				*wp = vPoints.back();
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);

		return ret;
	}

	//! Gets the current waypoint
	//获取当前的位点
	ReturnValue GetCurrentWaypoint(Waypoint *wp){
		ReturnValue ret = ERROR;
		pthread_mutex_lock(&mutexPath);
			if( (iCurrentWaypoint >= 0) && (iCurrentWaypoint < vPoints.size()) ){
				*wp = vPoints[iCurrentWaypoint];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);
		return ret;
	}

	//将第index个位点给wp
	ReturnValue GetWaypoint(int index, Waypoint *wp){
		ReturnValue ret = ERROR;

		pthread_mutex_lock(&mutexPath);
			if( (index >= 0) && ( index< vPoints.size() ) ){
				*wp = vPoints[index];
				ret = OK;
			}
		pthread_mutex_unlock(&mutexPath);
		return ret;
	}

	//! Gets the current Waypoint in the path
	//获取当前位点的id
	int GetCurrentWaypointIndex(){
		return iCurrentWaypoint;
	}

	//将当前的位点id设置位index
	ReturnValue SetCurrentWaypoint(int index){
		ReturnValue ret = ERROR;

		if(index < (vPoints.size() - 1)){
			pthread_mutex_lock(&mutexPath);
				iCurrentWaypoint = index;
			pthread_mutex_unlock(&mutexPath);
			ret = OK;
		}

		return ret;
	}

	//得到下一个waypoint的id
	void NextWaypoint(){
		pthread_mutex_lock(&mutexPath);
			iCurrentWaypoint++;
		pthread_mutex_unlock(&mutexPath);
	}

	//返回位点的数量
	int NumOfWaypoints(){
		return vPoints.size();
	}
	/*********************************************Waypoint基本的操作******************************/	

	//! Overloaded operator +=
	//重载加号，添加一个路径中的所有位点
	Path &operator+=(const Path &a){
		AddWaypoint(a.vPoints);
		return *this;
	}

	//! Cross product
	//.^2
	double dot2( Waypoint w, Waypoint v) {
		return (w.dX*v.dX + w.dY*v.dY);
	}

	//! Obtains the points for a quadratic Bezier's curve
	//! \param cp0 as player_pose2d_t, control point 0
	//!	\param cp1 as player_pose2d_t, control point 1
	//!	\param cp2 as player_pose2d_t, control point 2
	//!	\param t as float, [0 ... 1]
	//!	\return Point over the curve
	//获取二次贝塞尔曲线，三个控制点，t位控制参数取[0-1]之间
	Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t){
		Waypoint aux;

		//B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
		//二次贝塞尔曲线方程
		//Bx(t)
		aux.dX = (1.0-t)*(1.0-t)*cp0.dX + 2*t*(1.0-t)*cp1.dX + t*t*cp2.dX;
		//By(t)
		aux.dY = (1.0-t)*(1.0-t)*cp0.dY + 2*t*(1.0-t)*cp1.dY + t*t*cp2.dY;

		return aux;
	}

	//! Function that calculate the distance to deccelerate from target speed
	//! \param target_speed as double, speed on m/s
	//! \return distance on meters
	//根据目标速度计算需要的减速距离
	double DistForSpeed(double target_speed){
		if(target_speed > 1.0)
			return 2.0;
		else if(target_speed > 0.5)
			return 1.5;
		else return
			1.0;

	}

	//! Modifies and adds new waypoints to the route for improving the path
	//! \param distance as double, used for the calculation of the new points
	//! \return ERROR if Size is lower than 3, distance <= 0 or the waypoints has already been optimized
	//! \return OK
	/*为了优化路径添加新的位点和修改已有的位点
	参数distance用于计算新的点
	*/
	ReturnValue Optimize(double distance){
		int i, j=0;
		int a, b, c;
		int x = 0, y = 1, speed = 2;
		double mod_ab, mod_bc;
		double dAngle;
		Waypoint ab, bc, ba;
		double K= 0.0;
		vector <Waypoint> new_points;
		Waypoint aux;
		double Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0, Cx = 0.0, Cy = 0.0;
		Waypoint A, B, C;
		double Kt = 1.0 / BEZIER_CONTROL_POINTS;
		double dAuxSpeed = 0.0;
		double dMinDist = 0.0;	// Minica distancia a la que hay q frenar en funcion de la velocidad
		
		//判断是否已经优化
    if(bOptimized){	//Already optimizedpurepursuit_planner
			return OK;
		}
		//判断点容器中点的个数，如果小于2个或者距离为0，则不适合做优化
		if((vPoints.size() < 2) || (distance <= 0.0)){	
			return ERROR;
		}
		
		pthread_mutex_lock(&mutexPath);

			//如果容器中的点数为2
			if(vPoints.size() == 2){
				//将容器中最后一个元素再添加一次，变成三个元素了
				aux = vPoints[1];
				vPoints.push_back(aux); 
				if((vPoints[0].dX - aux.dX) == 0.0){//如果X坐标相同
					// 将第一、二个点的x坐标设置为相同    
					vPoints[1].dX = vPoints[0].dX;
					//将第二个点的y的距离设置到第一个点和第二个点中间
					vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0; 
				}else if((vPoints[0].dY - aux.dY) == 0.0){ // 如果y坐标相等
					 // 将x坐标设置为两个点的中间
					vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0;
					vPoints[1].dY = vPoints[0].dY;
				}else{ // 其它情况将x，y坐标都设置为两点的中间
					vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; 
					vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0;
				}
			}
			//在新位点的容器中添加两个新的点
			new_points.push_back(vPoints[0]);
			new_points.push_back(vPoints[1]);
			
/*********************************循环处理已有位点**********************************************************/
			for(i=2; i < vPoints.size(); i++){
				//三个控制点
				a = i-2;
				b = i-1;
				c = i;
				
				//位点的距离a,b,c的x,y坐标
				Ax = vPoints[a].dX;
				Ay = vPoints[a].dY;
				Bx = vPoints[b].dX;
				By = vPoints[b].dY;
				Cx = vPoints[c].dX;
				Cy = vPoints[c].dY;

				//计算a,b两点之间的距离
				ab.dX = Bx - Ax;
				ab.dY = By - Ay;
				mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);
				
				//计算b,c两点之间的距离
				bc.dX = Cx - Bx;
				bc.dY = Cy - By;
				mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
				
				//角度
				/*angle = acos((x1*x2 + y1*y2)/(sqrt(x1^2 + y1^2) * sqrt(x2^2 + y2^2)))
				<abc的角度，acos(angle) = (a^2 + b^2 -c^2)/(2 * a * b) */
				dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));

/**************************判断角度大于等于最小的贝塞尔角度******************************************/
				if(fabs(dAngle) >= MIN_ANGLE_BEZIER){
					//删除new_points最后一个点，也就是b点
					new_points.pop_back();
					
					// 如果角度大于等于45度，最大速度等级为MAX_SPEED_LVL2,否则为MAX_SPEED_LVL1
					if(fabs(dAngle) >= (Pi/4)){
						dAuxSpeed = MAX_SPEED_LVL2;
					}else
						dAuxSpeed = MAX_SPEED_LVL1;
					
/****************************如果中间的位点的速度绝对值大于最大速度************************************/
					if(fabs(vPoints[b].dSpeed) > dAuxSpeed){
						
						//如果速度方向相反，取反
						if(vPoints[b].dSpeed < 0.0)	// Cambiamos sentido de avance
							dAuxSpeed = -dAuxSpeed;
						
						//最小距离为减速距离
						dMinDist = DistForSpeed(fabs(vPoints[b].dSpeed));
						
				/***********如果ab节点之间的距离大于最小减速距离*************/
						if( mod_ab > dMinDist){
							//得到开始减速的点
							ba.dX = -ab.dX;
							ba.dY = -ab.dY;
							K = dMinDist / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);

							aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
							aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
							aux.dSpeed = vPoints[b].dSpeed;
							//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
							new_points.push_back(aux);
						}
						vPoints[b].dSpeed = dAuxSpeed; //将中间位点的速度设置为最大速度
					}
/*****************************如果ab节点之间的距离大于给定的距离，在中间插入新的点**************************/
					if(mod_ab > distance){
						//Lo creamos
						ba.dX = -ab.dX;
						ba.dY = -ab.dY;
						K = distance / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);

						aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
						aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
						aux.dSpeed = vPoints[b].dSpeed;
						//cout << "Nuevo punto en " << aux.dX << ", " << aux.dY << endl;
						new_points.push_back(aux);
					}
					//将b点添加到new_points中
					new_points.push_back(vPoints[b]);
					
/*****************************如果bc节点之间的距离大于给定的距离，在中间插入新的点**************************/
					if(mod_bc > distance){ 
						//Lo creamos
						K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
						aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
						aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
						aux.dSpeed = vPoints[b].dSpeed;
						new_points.push_back(aux);
					}

		/********************如果最小减速距离大于0***********************/
					if(dMinDist > 0.0) {
						//bc之间的距离大于1.0，插入辅助点
						if(mod_bc > 1.0){	
							K = 1.0 / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
							aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
							aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
							aux.dSpeed = vPoints[b].dSpeed;
							new_points.push_back(aux);
						}else{
							vPoints[c].dSpeed = vPoints[b].dSpeed;
						}
					}
					//添加位点c
					new_points.push_back(vPoints[c]);
				
				}else{//如果角度小于15度，直接添加位点c
					new_points.push_back(vPoints[c]);
				}
			}//所有位点处理结束
			
			//清除原来容器中的位点
			vPoints.clear();
			
/******************将新的位点添加到位点容器中，并用贝塞尔曲线优化************************/
			// BEZIER
			vPoints.push_back(new_points[0]);
			vPoints.push_back(new_points[1]);
			
			//对new_points中的点进行贝塞尔曲线优化
			for(i=2; i < new_points.size(); i++){	// Segunda pasada, aproximamos los giros a curvas de Bezier
				a = i-2;
				b = i-1;
				c = i;

				Ax = new_points[a].dX;
				Ay = new_points[a].dY;
				Bx = new_points[b].dX;
				By = new_points[b].dY;
				Cx = new_points[c].dX;
				Cy = new_points[c].dY;

				ab.dX = Bx - Ax;
				ab.dY = By - Ay;
				mod_ab = sqrt(ab.dX * ab.dX + ab.dY * ab.dY);

				bc.dX = Cx - Bx;
				bc.dY = Cy - By;
				mod_bc = sqrt(bc.dX * bc.dX + bc.dY * bc.dY);

				dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));
				
				//判断最小角度是否>=最小的贝塞尔角
				if(fabs(dAngle) >= MIN_ANGLE_BEZIER){
					Waypoint aux_wp;
					double t, aux_speed;

					A = new_points[a];
					B = new_points[b];
					C = new_points[c];

					aux_speed = new_points[b].dSpeed; //获取中间点的速度
					vPoints.pop_back();//删除最后一个点
					
					//用贝塞尔曲线优化路径，优化的点数为5个
					for(int j=1; j <= BEZIER_CONTROL_POINTS; j++) {
						t = (double) j * Kt;
						aux_wp = PosOnQuadraticBezier(A, B, C,  t);
						aux_wp.dSpeed = aux_speed;
						vPoints.push_back(aux_wp);
					}
				}else{//如果角度小于贝塞尔角度，则直接将位点添加进去
					vPoints.push_back(new_points[c]);
				}
			}
			//当前位点id初始化为0
			iCurrentWaypoint = 0;

		//解锁，优化完成，清除new_points容器
		pthread_mutex_unlock(&mutexPath);
		bOptimized  = true;
		new_points.clear();
		
		return OK;
	}
	//! Prints all the waypoints
	//打印所有的位点
	void Print(){
		cout << "Path::Print: Printing all the waypoints..." << endl;
		if(vPoints.size() > 0){
			for(int i = 0; i < vPoints.size(); i++){
				cout << "(" << i << ")\t " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
			}
		}else
			cout << "Path::Print: No waypoints..." << endl;
	}
};

/***********************************purepursuit_planner_node**********************************************/

class purepursuit_planner_node: public Component
{

private:
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq_;
	//! constant for Purepursuit
	double Kr;

    //! Variable lookahead
    //! 可变前瞻量
    double dLookAhead; 
     //! Object with the current path that the robot is following
    //机器人当前跟踪的路径
    Path pathCurrent;
    //! Object with the path that is being filled
    //正在被填充的路径
    Path pathFilling;
    //! Vector with next paths to follow
    //路径队列
    queue <Path> qPath;
    //! current robot's position
    //当前的机器恩位置
    geometry_msgs::Pose2D pose2d_robot;
    //! current robot's odometry
    //当前机器人的里程计信息
    nav_msgs::Odometry odometry_robot;
    //! current robot's linear speed
    //当前机器人的线性速度
    double dLinearSpeed;
    //! Lookahead bounds
    //机器人前瞻量的范围
    double d_lookahear_min_, d_lookahear_max_;
    //! Distance from the robot center to the wheel's center
    //机器人的中心到轮子的距离
    double d_dist_wheel_to_center_;
    //! Max allowed speed
    //最大的允许速度
    double max_speed_;
    //! Flag to enable/disable the motion
    //
    bool bEnabled;
    //! Flag to cancel the following path
    bool bCancel;
    
    bool Avoid;
    //! Mode for reading the position of the robot ("ODOM", "MAP")
    //读取机器人的位置的方式，odom坐标还是map坐标
    std::string position_source_;
    //! Mode in numeric format
    //数字形式的mode
    unsigned int ui_position_source;

	//////// ROS
	//! Publish to cmd vel (Ackermann)
	//! It will publish into command velocity (for the robot)
	//发布速度命令
	ros::Publisher vel_pub_;
	
	ros::Publisher path_pub_;
	//! publish the transformation between map->base_link
	//发布map和base_link之间的TF转换
	ros::Publisher tranform_map_pub_;
	//! it subscribes to /odom
	//订阅/odom消息
	ros::Subscriber odom_sub_;
	
	ros::Subscriber laser_sub_;
	
	//得到机器人的初始位置
	//ros::Subscriber initialpose_sub_;
	//! Topic name to read the odometry from
	//从这个odom话题读取odom消息
	std::string odom_topic_;
	
	std::string path_topic_;
	
	std::string base_link_topic_;
	
	std::string laser_topic_;

	//发布速度和位置的命令的话题
	std::string cmd_topic_vel_;
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published odom
	//诊断odom的发布频率
	diagnostic_updater::TopicDiagnostic *updater_diagnostic_odom;
	//! General status diagnostic updater
	//通用状态诊断更新
	diagnostic_updater::Updater updater_diagnostic;
	//! Diagnostics min & max odometry freq
	//最小或者最大的odom频率
	double min_odom_freq, max_odom_freq;
	//! Saves the time whenever receives a command msg and a transform between map and base link (if configured)
	//当接收到一个命令消息和一个map到bae link之间的TF转换时，保存这个时间
	ros::Time last_command_time, last_map_time;
	// ACTIONLIB
	//动作库
	actionlib::SimpleActionServer<planner_msgs::GoToAction> action_server_goto;
	planner_msgs::GoToFeedback goto_feedback;
	planner_msgs::GoToResult goto_result;
	planner_msgs::GoToGoal goto_goal;
	// TFs
	//TF转换
	tf::TransformListener listener;
	tf::StampedTransform transform;
	// SERVICES
	//关于激光雷达的服务
	//! service name to enable disable lasers
	std::string name_sc_enable_front_laser_, name_sc_enable_back_laser_;
	//! Service to enable/disable front laser
	ros::ServiceClient sc_enable_front_laser_;
	//! Service to enable/disable back laser
	ros::ServiceClient sc_enable_back_laser_;

public:

	/*!	\fn summit_controller::purepursuit_planner()
	 * 	\brief Public constructor
	*/
	//顶级控制器，构造函数
	purepursuit_planner_node(ros::NodeHandle h) : 
	    node_handle_(h), 
	    private_node_handle_("~"),
	    desired_freq_(20.0), //频率100的时候会出现停止信号响应延迟严重的现象
	    Component(desired_freq_),
	    action_server_goto(node_handle_, ros::this_node::getName(), false)
	// boost::bind(&purepursuit_planner_node::executeCB, this, _1), false)
	{
		bRunning = false;

		//ROS设置
		ROSSetup();

		dLookAhead = d_lookahear_min_;
		dLinearSpeed = 0;
		pose2d_robot.x = pose2d_robot.y = pose2d_robot.theta = 0.0;
		bEnabled = true;
		bCancel = false;

		sComponentName.assign("purepursuit_planner_node");
		iState = INIT_STATE;
	}

	/*!	\fn purepursuit_planner::~purepursuit_planner()
	 * 	\brief Public destructor
	*/
	~purepursuit_planner_node(){

	}

	/*!	\fn oid ROSSetup()
	 * 	\brief Setups ROS' stuff
	*/
	//ROS设置
	void ROSSetup(){
		//私有参数(param(参数名，参数值，默认参数值))
		private_node_handle_.param<std::string>("odom_topic", odom_topic_, "/odom");
		private_node_handle_.param("cmd_topic_vel", cmd_topic_vel_, std::string("/cmd_vel"));
		private_node_handle_.param("d_lookahear_min", d_lookahear_min_, D_LOOKAHEAD_MIN);
		private_node_handle_.param("d_lookahear_max", d_lookahear_max_, D_LOOKAHEAD_MAX);
		private_node_handle_.param("d_dist_wheel_to_center", d_dist_wheel_to_center_, D_WHEEL_ROBOT_CENTER);
		private_node_handle_.param("max_speed", max_speed_, MAX_SPEED);
		private_node_handle_.param("kr", Kr, AGVS_DEFAULT_KR);
		private_node_handle_.param<std::string>("position_source", position_source_, "ODOM");
		private_node_handle_.param("desired_freq", desired_freq_, desired_freq_);
		private_node_handle_.param("path_topic", path_topic_, std::string("/path"));
		//private_node_handle_.param("initialpose_topic", initialpose_topic_, std::string("/initialpose"));
		private_node_handle_.param("base_link_topic",base_link_topic_, std::string("/base_link"));
		private_node_handle_.param("laser_topic",laser_topic_, std::string("/scan"));

		// From Component class
		//线程控制频率
		threadData.dDesiredHz = desired_freq_;

		if(position_source_ == "MAP")
			ui_position_source = MAP_SOURCE;
		else
			ui_position_source = ODOM_SOURCE;

		// Publish through the node handle Twist type messages to the guardian_controller/command topic
		//发布速度消息
		vel_pub_ = private_node_handle_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);

		//发布优化后的路径
		path_pub_ = private_node_handle_.advertise<nav_msgs::Path>(path_topic_, 10);
		
		//广播map的TF数据
		if(ui_position_source == MAP_SOURCE)
			tranform_map_pub_ = private_node_handle_.advertise<geometry_msgs::TransformStamped>("map_location", 100);

		odom_sub_ = private_node_handle_.subscribe<nav_msgs::Odometry>(odom_topic_, 
																					1,
											&purepursuit_planner_node::OdomCallback,
																				this);
		//订阅激光的数据
		laser_sub_ = private_node_handle_.subscribe<sensor_msgs::LaserScan>(laser_topic_,
																						1,
													&purepursuit_planner_node::LaserAvoid,
																					this);
		
		// Diagnostics
		//设置硬件ID
		updater_diagnostic.setHardwareID("PurePursuit-Planner");
		// Topics freq control
		min_odom_freq = 5.0;
		max_odom_freq = 50.0;
		updater_diagnostic_odom = new diagnostic_updater::TopicDiagnostic(odom_topic_, 
																	updater_diagnostic,
	diagnostic_updater::FrequencyStatusParam(&min_odom_freq, &max_odom_freq, 0.1, 10),
								diagnostic_updater::TimeStampStatusParam(0.001, 0.1));
		//动作服务器
		action_server_goto.registerGoalCallback(boost::bind(&purepursuit_planner_node::GoalCB, this));
		action_server_goto.registerPreemptCallback(boost::bind(&purepursuit_planner_node::PreemptCB, this));

		/*ROS的消息*/
		ROS_INFO("%s::ROSSetup(): \n\
				laser_topic = %s,\n\
				base_link_topic = %s,\n\
				odom_topic = %s,\n\
				command_topic_vel = %s,\n\
				position source = %s, \n\
				desired_hz=%.1lf, \n\
				min_lookahead = %.1lf,\n\
				max_lookahead = %.1lf,\n\
				kr = %.2lf", 
				sComponentName.c_str(), 
				laser_topic_.c_str(),
				base_link_topic_.c_str(),
				odom_topic_.c_str(),
				cmd_topic_vel_.c_str(), 
				position_source_.c_str(), 
				desired_freq_, 
				d_lookahear_min_, 
				d_lookahear_max_, 
				Kr);
	}


	/*!	\fn ReturnValue Setup()
	 * 	\brief
	*/
	//设置
	ReturnValue Setup(){
		// Checks if has been initialized
		//判断是否已经初始化
		if(bInitialized){
			ROS_INFO("purepursuit_planner::Setup: Already initialized");
			return INITIALIZED;
		}

		// Starts action server
		//启动动作服务器
		action_server_goto.start();

		bInitialized = true;

		return OK;
	}


	/*! \fn int ReadAndPublish()
	 * Reads data and publish several info into different topics
	*/
	//读取信息诊断并发布
	int ReadAndPublish()
	{
		//updater_diagnostic_odom->tick(ros::Time::now());
		updater_diagnostic.update();
		return(0);
	}

	/*!	\fn ReturnValue Start()
	 * 	\brief Start Controller
	*/
	//开始控制
	ReturnValue Start(){
		
		if(bRunning){
			ROS_INFO("agvs_controller::Start: the component's thread is already running");
			return THREAD_RUNNING;
		}
		bRunning = true;
		return OK;
	}

	/*!	\fn ReturnValue Stop()
	 * 	\brief Stop Controller
	*/
	//停止控制器
	ReturnValue Stop(){

		if(!bRunning){
			ROS_INFO("agvs_controller::Stop: Thread not running");

			return THREAD_NOT_RUNNING;
		}

		bRunning = false;

		return OK;
	}

	/*! \fn void ControlThread()
	*/
	//控制线程
	void ControlThread()
	{
		ROS_INFO("purepursuit_planner::ControlThread(): Init");
		ros::Rate r(desired_freq_);  // 20.0

		// while(node_handle_.ok()) {
		while(ros::ok()){//循环处理各种状态
			//判断状态
			switch(iState){
				
				case INIT_STATE://初始化状态
					InitState();
				break;

				case STANDBY_STATE://待命状态
					StandbyState();
				break;

				case READY_STATE://准备状态
					ReadyState();
				break;

				case SHUTDOWN_STATE:
					ShutDownState();
				break;

				case EMERGENCY_STATE:
					EmergencyState();
				break;

				case FAILURE_STATE:
					FailureState();
				break;

			}

			AllState();//所有状态都需要执行的

			ros::spinOnce();
			r.sleep();
		}
		
		ShutDownState();

		ROS_INFO("purepursuit_planner::ControlThread(): End");

	}

	/*!	\fn void InitState()
	*初始化状态*/
	void InitState(){
		//ROS_INFO("purepursuit_planner::InitSate:");
		//判断是否已经初始化并且启动
		if(bInitialized && bRunning){
			//是否odom数据正常
			if(CheckOdomReceive() == 0){
				//切换到STANDBY_STATE状态
				SwitchToState(STANDBY_STATE);
				//ROS_INFO("q");
				
			}
		}else{
			if(!bInitialized)
				//设置
				Setup();
			if(!bRunning)
				//启动
				Start();
		}

	}

	/*!	\fn void StandbyState()
	*///独立运行状态
	void StandbyState(){
		//如果odom数据有问题
		if(CheckOdomReceive() == -1)
			//转换到紧急状态
			SwitchToState(EMERGENCY_STATE);
		else{//否则判断是否能并且没有取消路径
			if(bEnabled && !bCancel ){
				//如果路径大于0，或者合成路径成功
				if(pathCurrent.Size() > 0 || MergePath() == OK){
					ROS_INFO("%s::StandbyState: route available", sComponentName.c_str());
					//切换到准备状态
					SwitchToState(READY_STATE);
				}
			}
		}
	}

	/*!	\fn void ReadyState()
	*///准备状态
	void ReadyState(){
		//判断odom是否由问题
		if(CheckOdomReceive() == -1){
			SetRobotSpeed(0.0, 0.0);
			SwitchToState(EMERGENCY_STATE);
			return;
		}
		if(!bEnabled){//如果没有使能
			ROS_INFO("%s::ReadyState: Motion is disabled", sComponentName.c_str());
			SetRobotSpeed(0.0, 0.0);
			//切换到STANDBY_STATE状态
			SwitchToState(STANDBY_STATE);
			return;
		}//判断是否已经取消路径
		if(bCancel){
			ROS_INFO("%s::ReadyState: Cancel requested", sComponentName.c_str());
			SetRobotSpeed(0.0, 0.0);
			//切换到STANDBY_STATE状态
			SwitchToState(STANDBY_STATE);
			return;
		}
		
		//开始追踪路径
		int ret = PurePursuit();
		
		if(ret == -1){
			ROS_ERROR("%s::ReadyState: Error on PurePursuit", sComponentName.c_str());
			bCancel = true;	//Activates the flag to cancel the mision
			SetRobotSpeed(0.0, 0.0);
			goto_result.route_result = -1;
			goto_feedback.percent_complete = 100.0;	// Set the percent to 100% to complete the action
			SwitchToState(STANDBY_STATE);
		}else if(ret == 1){
			ROS_INFO("%s::ReadyState: Route finished", sComponentName.c_str());
			SetRobotSpeed(0.0, 0.0);
			goto_result.route_result = 0;
			goto_feedback.percent_complete = 100.0;	// Set the percent to 100% to complete the action
			SwitchToState(STANDBY_STATE);
		}
		//停止机器人
		//if()
		//SetRobotSpeed(0.0, 0.0);
		// We have to update the percent while the mision is ongoing
		//当任务在执行的时候需要更新百分数

	}

	/*! \fn void UpdateLookAhead()
	*   \brief Updates (little by little) the variable lookahead depending of the current velocity
	*/
	//根据当前的速度一点点更新lookahead变量

	void UpdateLookAhead(){
	double aux_lookahead = fabs(dLinearSpeed);//线速度的绝对值
    double desired_lookahead = 0.0;//期望前瞻量赋０
    double inc = 0.01;	//增量

    //限制期望速度在最小和最大值之间
    /*
		if(aux_lookahead < d_lookahear_min_)
			desired_lookahead = d_lookahear_min_;
		else if(aux_lookahead > d_lookahear_max_)
			desired_lookahead = d_lookahear_max_;
		else{
			desired_lookahead = aux_lookahead;
    }*/
    //等价
    /********************************/
    if(aux_lookahead < d_lookahear_min_)
      aux_lookahead = d_lookahear_min_;
    else if(aux_lookahead > d_lookahear_max_)
      aux_lookahead = d_lookahear_max_;

      desired_lookahead = aux_lookahead;
    /********************************/

    //精确的到毫米
		if((desired_lookahead - 0.001) > dLookAhead){
			dLookAhead+= inc;
		}else if((desired_lookahead + 0.001) < dLookAhead)
			dLookAhead-= inc;
  }
  /**************************************************************************************/
	/*! \fn double Dot2( double x1, double y1, double x2, double y2)
	*   \brief Obtains vector cross product w x v
	*   \return w.x * v.x + w.y * w.y
	*///获取点乘积
	double Dot2( double x1, double y1, double x2, double y2) {
		return (x1*x2 + y1*y2); // cross product
	}


	/*! \fn double Dist(double x1, double y1, double x2, double y2)
	*   \brief obtains distance between points p1 and p2
	*///得到两个点之间的距离hypot
	double Dist(double x1, double y1, double x2, double y2) {
		double diff_x = (x2 - x1);
		double diff_y = (y2 - y1);
		return sqrt( diff_x*diff_x + diff_y*diff_y );
	}

	/*! \fn double DistP2S( Odometry current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
	 *  \brief obtains distance between the current position and segment s0->s1, and returns the point
	 *	Return: the shortest distance from p to s (utm points) and the point
	 *	of the segment that gives the shortest distance
	*///计算从当前点到段s0->s1的最短距离，并返回此时s0->s1上的点
	double DistP2S( geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb){
		double vx,vy, wx, wy;

		double c1, c2, di, b;

		//s0,s1之间的x，y坐标差
		vx = s1.dX - s0.dX;
		vy = s1.dY - s0.dY;

		wx = current_position.x - s0.dX;
		wy = current_position.y - s0.dY;

		c1 = Dot2( wx, wy, vx, vy );

		if ( c1 <= 0.0 ) {
			di = Dist(current_position.x, current_position.y, s0.dX, s0.dY);
			Pb->dX = s0.dX;
			Pb->dY = s0.dY;
			return di;
		}

		c2 = Dot2(vx,vy, vx, vy);

		if ( c2 <= c1 ) {
			//printf("kanban::DistP2S: c2 <= c1\n");
			di = Dist(current_position.x, current_position.y, s1.dX, s1.dY);
			Pb->dX = s1.dY;
			Pb->dY = s1.dY;
			return di;
		}

		b = c1 / c2;
		Pb->dX = s0.dX + b * vx;
		Pb->dY = s0.dY + b * vy;

		di = Dist(current_position.x, current_position.y, Pb->dX, Pb->dY);

		return di;
	}


	/*! \fn ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp	)
	 *  \brief Returns a point in a distance dlookahead on the path
	 *  \return OK
	 *  \return ERROR
  *///找到路径中的下一个点
	ReturnValue PointDlh(geometry_msgs::Pose2D current_position, geometry_msgs::Pose2D *wp) {
	int i,j=0,k;
	double dmin, d, d1, d2, *d_seg;
	double t;
	geometry_msgs::Pose2D target_position;//目标位置
	Waypoint s0, s1, Pb, Pb1;

	int size = pathCurrent.NumOfWaypoints();//当前路径中的位点数目

	d_seg = new double[size]; //储存每两个位点之间的线段的
	
		
	// 1- Find closest segment，
	//找到离当前位置最近的线段
	dmin = 100000000;

    //在所有的位点段上，求距离当前位置最近的位点段，并得到最近线段上较小的位点ID
	for(i = pathCurrent.GetCurrentWaypointIndex(); i < (size -1); i++) {
     //获取连续的两个位点
     if( (pathCurrent.GetWaypoint(i, &s0) == OK) &&  (pathCurrent.GetWaypoint(i+1, &s1) == OK) ){
        //s0和s1之间的距离
        d_seg[i] = Dist(s0.dX, s0.dY, s1.dX, s1.dY);
        //最短的距离
        d = DistP2S(current_position, s0, s1, &Pb1);		// Pb1 closest point on segment，Pb1是s0,s1上距离当前点最近的点
        //如果最小距离小于允许的距离
		if (d < dmin) {
			Pb.dX = Pb1.dX;  // not the same as Pb=Pb1 !
			Pb.dY = Pb1.dY;
			dmin = d;
			j = i;      // j : index to closest segment
		}
				//ROS_INFO("PointDlh. Distance to segment %d(%.2lf, %2.lf)->%d(%.2lf, %2.lf) = %.3lf,point (%.3lf, %.3lf) (DMIN = %.3lf)", i,
				//s0.dX, s0.dY, i+1, s1.dX, s1.dY, d, Pb1.dX, Pb1.dY, dmin);
		}else{
        ROS_ERROR("%s::PointDlh: Error Getting waypoints",sComponentName.c_str());
        return ERROR;
      }
	}
		
	//ROS_INFO("PointDlh:: Current waypoint index %d, next %d",pathCurrent.GetCurrentWaypointIndex(), j);
	//设置当前的位点ID为最近线段上的较小ID号
	if(pathCurrent.GetCurrentWaypointIndex() != j){
		// Sets the waypoint where the robot is at the moment
		if(pathCurrent.SetCurrentWaypoint(j) == ERROR){
			ROS_ERROR("%s::PointDlh: Error setting current waypoint to %d", sComponentName.c_str(), j);
			return ERROR;
		}else{ // OK
			//ROS_INFO("PointDlh:: Changing waypoint to %d", j);
				if(j == (size - 2)){	//当前的位点是倒数第二个位点
					Waypoint w_last, w_before_last;
					pathCurrent.GetCurrentWaypoint(&w_before_last);	// Penultimo waypoint
					pathCurrent.BackWaypoint(&w_last);				// Ultimo waypoint
					// Distancia maxima = distancia entre el punto actual y el penultimo punto + más la distancia entre los dos ultimos puntos + un valor constante
					//dMaxDistance = Dist(w_before_last.dX, w_before_last.dY, odomWhenLastWaypoint.px, odomWhenLastWaypoint.py) + Dist(w_last.dX, w_last.dY, w_before_last.dX, w_before_last.dY) + 0.1;
					//ROS_INFO("%s::PointDlh: Penultimo punto. Robot en (%.3lf, %.3lf, %.3lf). Distancia máxima a recorrer = %.3lf m ", sComponentName.c_str(), odomWhenLastWaypoint.px, odomWhenLastWaypoint.py, odomWhenLastWaypoint.pa, dMaxDistance);
				}
			}
	}

	// 2-Find segment ahead in dlookahead
    //在dlokhead范围内找到前面的segment
    //得到下一个点，赋值给s1
	if( pathCurrent.GetNextWaypoint(&s1) != OK ){
		ROS_ERROR("%s::PointDlh: Error getting next waypoint %d", sComponentName.c_str(), j);
		return ERROR;
	}
    //计算Pb(最近的点)和下一个点之间的距离
	d1 = Dist(Pb.dX, Pb.dY, s1.dX, s1.dY);

    k = j; // k : index of D_ point segment
    //当位于第二个位点之前，找到刚好大于等于前瞻量的下一个位点
    while ( (d1 < dLookAhead) && ( (k+1) < (size - 1) ) ) {
		// searched point on this segment
		k = k + 1;
		d1 = d1 + d_seg[k];
    }

	// 3- Obtain t parameter in the segment
    // t parameter of segment k
    //在第k段上找到刚好等于前瞻量dLookAhead的点
    d2 = ( d1 - dLookAhead );//超过dLookAhead多少
	t = (d_seg[k] - d2) / d_seg[k]; 
	
	// 4- Obtain point with t parameter
    //当前点和下一个点
    if( (pathCurrent.GetWaypoint(k, &s0) == OK) && (pathCurrent.GetWaypoint(k + 1, &s1)== OK)){
			target_position.x = s0.dX + ( s1.dX - s0.dX )*t;
			target_position.y = s0.dY + ( s1.dY - s0.dY )*t;
      double angle_segment = atan2(s1.dY - s0.dY, s1.dX - s0.dX);//角度

      //角度(弧度)正常化
			radnorm(&angle_segment);

			target_position.theta = angle_segment;
			*wp = target_position;

			delete d_seg;

			return OK;
		}else{
			ROS_ERROR("%s::PointDlh: Error getting next waypoint %d", sComponentName.c_str(), j);
			return ERROR;
		}

	}


	/*!	\fn int PurePursuit()
	 * \brief High level control loop in cartesian coordinates
	 * obtains desiredSpeedMps and desiredPhiEffort according to
	 * the robot location and the path defined by the waypoints
	 *  \return 0 if the iteration is OK
	 *  \return -1 if there's a problem
	 *  \return 1 if the route finishes
	 */
	 /*在迪卡尔坐标系中的高水平的控制回路
	 根据机器人路径中的位置点获取的期望速度和方向
	 */
	int PurePursuit(){
		double dx, dy, x1, y1;
		double curv, yaw;
		double Kd = 1.1; // don't increase! 
		Waypoint last_waypoint, next_waypoint;
		double dAuxSpeed = 0.0;
		double dth;
		double aux = 0.0, dDistCovered = 0.0;
		int ret = 0;

		//获取当前的机器人位姿
		geometry_msgs::Pose2D current_position = this->pose2d_robot;
		//机器人的下一个位姿
		geometry_msgs::Pose2D next_position;

		//位点数至少两个
		if(pathCurrent.NumOfWaypoints() < 2)	{
			ROS_ERROR("%s::PurePursuit: not enought waypoints", sComponentName.c_str());
			return -1 ;
		}
		//航向角为当前的
		yaw = current_position.theta;

		//Updates the lookahead depending of the current velocity
		//根据当前的速度更新前瞻量
		UpdateLookAhead();

		//根据机器人的当前位置获取路径上的下一个目标点
		if(PointDlh(current_position, &next_position) != OK){
			ROS_ERROR("%s::PurePursuit: Error getting next point in the route", sComponentName.c_str());
			return -1;
		}

		//弯曲
		//将世界坐标系的距离关系转换为机器人的坐标系
		dx = current_position.x - next_position.x;
		dy = current_position.y - next_position.y;
		x1 = cos(yaw)*dx + sin(yaw)*dy; //Original
		y1 = -sin(yaw)*dx + cos(yaw)*dy;
		
		//ROS_INFO("dx : %f, dy : %f) ",dx, dy);
        //判断曲率半径
		if ((x1*x1 + y1*y1) == 0)
			curv = 0.0;
		else
			curv = (2.0 / (x1*x1 + y1*y1)) * -y1;  		//Original

		//得到当前路径的最后一个点
		if(pathCurrent.BackWaypoint(&last_waypoint) == ERROR){
			ROS_ERROR("%s::PurePursuit: Error getting the last point in the path", sComponentName.c_str());
			return -1;
		}
		//得到当前点到最后一个点之间的距离
		double dAuxDist = Dist(current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);	

		//得到下一个点
		if(pathCurrent.GetNextWaypoint(&next_waypoint) == ERROR){
			ROS_ERROR("%s::PurePursuit: Error getting next waypoint in the path", sComponentName.c_str());
			return -1;
		}
		//得到下一个点的速度
		dAuxSpeed = next_waypoint.dSpeed;

		//根据速度限制设置速度
		if(fabs(dAuxSpeed) > max_speed_){
			if(dAuxSpeed > 0)
				dAuxSpeed = max_speed_;
			else
				dAuxSpeed = -max_speed_;
		}

		if(dAuxDist <= AGVS_SECOND_DECELERATION_DISTANCE)	{
			if( (dAuxSpeed < 0.0) && (dAuxSpeed < -AGVS_SECOND_DECELERATION_MAXSPEED) )
				dAuxSpeed = -AGVS_SECOND_DECELERATION_MAXSPEED;
			else if( (dAuxSpeed > 0.0) && (dAuxSpeed > AGVS_SECOND_DECELERATION_MAXSPEED) )
				dAuxSpeed = AGVS_SECOND_DECELERATION_MAXSPEED;
		}else if(dAuxDist <= AGVS_FIRST_DECELERATION_DISTANCE) {
			if( (dAuxSpeed < 0.0) && (dAuxSpeed < AGVS_FIRST_DECELERATION_MAXSPEED))
				dAuxSpeed = -AGVS_FIRST_DECELERATION_MAXSPEED;
			else if( (dAuxSpeed > 0.0) && (dAuxSpeed > AGVS_FIRST_DECELERATION_MAXSPEED) )
				dAuxSpeed = AGVS_FIRST_DECELERATION_MAXSPEED;
		}

	if(!Avoid){
		//设置机器人的速度和转向角度
		SetRobotSpeed(dAuxSpeed, 1.0/curv);
		
	}else{//出现人等障碍物时暂停
		SetRobotSpeed(0, 0);
		
	}

    //当机器人到达最后一个点时，检测它距离最后点的距离
	if( pathCurrent.GetCurrentWaypointIndex() >= (pathCurrent.NumOfWaypoints() - 2) ){
			ret = -1;
			//计算当前位置到最后位点的距离
			double ddist2 = Dist( current_position.x, current_position.y, last_waypoint.dX, last_waypoint.dY);

			//如果距离小于允许的误差距离，则停止机器人
			if (ddist2 < WAYPOINT_POP_DISTANCE_M) {

				SetRobotSpeed(0.0, 0.0);
				
				//调用位置姿态调整函数
				//SetEndPose();
				
				
				
				
				ROS_INFO("%s::PurePursuit: target position reached (%lf, %lf, %lf). Ending current path", 
						sComponentName.c_str(), 
						current_position.x, 
						current_position.y,
						current_position.theta*180.0/Pi);
						
				pathCurrent.Clear();
				return 1;
			}
		}
		
		//发布优化后的路径pathCurrent.GetWaypoint(i, &s0)
		nav_msgs::Path better_path;
		better_path.poses.resize(pathCurrent.NumOfWaypoints());
		Waypoint s;
		for(int i = 0 ;i < pathCurrent.NumOfWaypoints(); i++){
			pathCurrent.GetWaypoint(i, &s);
			better_path.header.frame_id = odom_topic_;
			better_path.poses[i].pose.position.x = s.dX;
			better_path.poses[i].pose.position.y = s.dY;
			
		}
		path_pub_.publish(better_path);
		
		return 0;
	}

	/*!	\fn void CancelPath()
	 * Removes all the waypoints introduced in the system
	*/
	//取消路径并移除路径中的所有点
	void CancelPath(){

		pathCurrent.Clear();	// Clears current path
		pathFilling.Clear();	// Clears the auxiliary path
		while(!qPath.empty())	// Clears the queue of paths
			qPath.pop();

		bCancel = false;
		// Cancels current action
		ROS_INFO("%s::CancelPath: action server preempted", sComponentName.c_str());
		//服务器被取代了
		action_server_goto.setPreempted();
	}

	/*!	\fn void SetRobotSpeed()
	*/
	//设置机器人的速度
  void SetRobotSpeed(double speed, double radius){

    geometry_msgs::Twist ref_msg;
    
    ref_msg.linear.x = speed;
    ref_msg.linear.y = 0.0;
    ref_msg.linear.z = 0.0;
    ref_msg.angular.x = 0.0;
    ref_msg.angular.y = 0.0;
    ref_msg.angular.z = (radius == 0.0) ? 0 : AGVS_SPEED_K * speed / radius ;
    if(ref_msg.angular.z >= AGVS_TURN_SPEED){
		ref_msg.angular.z = AGVS_TURN_SPEED;
	}
	vel_pub_.publish(ref_msg);
	if((radius == 0) && (speed == 0)){
		vel_pub_.publish(ref_msg);
		vel_pub_.publish(ref_msg);
		vel_pub_.publish(ref_msg);
		vel_pub_.publish(ref_msg);
		vel_pub_.publish(ref_msg);
		}
	}
	
	/*!	\fn void ShutDownState()
	*///关机状态
	void ShutDownState(){
		if(bRunning)
			Stop();
		else if(bInitialized)
			ShutDown();
	}

	/*!	\fn void EmergencyState()
	*///紧急状态
	void EmergencyState(){
		if(CheckOdomReceive() == 0){
			SwitchToState(STANDBY_STATE);
			return;
		}

	}

	/*!	\fn void FailureState()
	*///失败状态
	void FailureState(){

	}

	/*!	\fn void AllState()
	所有的状态都需要*/
	void AllState(){
		// 只有当我们使用地图进行地位时
		if(ui_position_source == MAP_SOURCE){
			try{
				//得到map与base_link之间的TF转换关系
				listener.lookupTransform("/map", base_link_topic_, ros::Time(0), transform);
				geometry_msgs::TransformStamped msg;
				tf::transformStampedTFToMsg(transform, msg);//将TF转换为消息
				//得到机器人的位置姿态
				pose2d_robot.x = msg.transform.translation.x;
				pose2d_robot.y = msg.transform.translation.y;
				pose2d_robot.theta = tf::getYaw(msg.transform.rotation);
				// Safety check
				last_map_time = ros::Time::now();
				msg.header.stamp = last_map_time;
				tranform_map_pub_.publish(msg);//发布map->base_link的TF消息
			}catch (tf::TransformException ex){
				//ROS_ERROR("%s::AllState: %s", sComponentName.c_str(), ex.what());
			}
		}
		AnalyseCB();	// 检查动作服务器的状态
		ReadAndPublish();	// Reads and publish into configured topics
		if(bCancel)		// Performs the cancel in case of required
			CancelPath();
	}

	/*! \fn void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
		* Receives odom values
	*/
	/*接受odom的数据*/
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_value)
	{
		// Safety check
		last_command_time = ros::Time::now();

		// If we want to use the odom source, subscribes to odom topic
		if(ui_position_source == ODOM_SOURCE){
			// converts the odom to pose 2d
			//得到机器人的位置姿态
			pose2d_robot.x = odom_value->pose.pose.position.x;
			pose2d_robot.y = odom_value->pose.pose.position.y;
			pose2d_robot.theta = tf::getYaw(odom_value->pose.pose.orientation);
		}
		// Copies the current odom
		//复制机器人的odom信息
		odometry_robot = *odom_value;
		// Gets the linear speed
		//得到线性速度
		dLinearSpeed = odometry_robot.twist.twist.linear.x;
	}

	/*! \fn int CheckOdomReceive()
		* Checks whether or not it's receiving odom values and/or map transformations
		* \return 0 if OK
		* \return -1 if ERROR
	*/
	//检查是否接受到odom或者map的TF转换
	int CheckOdomReceive()
	{
		// Safety check
		//时间间隔与time out的比较
		if((ros::Time::now() - last_command_time).toSec() > ODOM_TIMEOUT_ERROR){
			return -1;
		}
		else{
			if( ui_position_source == MAP_SOURCE and ((ros::Time::now() - last_map_time).toSec() > MAP_TIMEOUT_ERROR)){
				return -1;
			}
			else return 0;
		}

	}


	void executeCB(const planner_msgs::GoToGoalConstPtr &goal)
	{

	}

	/*! \fn int CalculateDirectionSpeed(Waypoint target_position)
	*	\brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
	*	\return 1 si el sentido es positivo
	*	\return -1 si el sentido es negativo
	*/
	/*计算速度方向
	 * １ 是正向速度
	 * -1 是负速度
	 * */
	int CalculateDirectionSpeed(Waypoint target_position){
		int ret = 1;
		//得到机器人的当前的位置姿态
		double alpha = pose2d_robot.theta;
		double x =	pose2d_robot.x, y = pose2d_robot.y;
		double ux, uy, vx, vy;
		double beta = 0.0;
		static int last_direction = 0;
		static double pi_medios = M_PI / 2.0,
						  max_diff = 5.0 * M_PI / 180.0;//最大的差角５度
		int iCase = 0;

		//如果当前位置和目标位置相同
		if( (target_position.dX == x) && (target_position.dY == y) ){
			return 0;
		}

		ux = cos(alpha);
		uy = sin(alpha);

		// 计算目标位置和当前的ｘ，ｙ之间的坐标差
		vx = target_position.dX - x;
		vy = target_position.dY - y;

		/*向量法计算两个向量之间的夹角
		机器人再当前点的航向角与当前点到目标点之间的连线的夹角*/
		beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );

		//如果夹角的绝对值小于90度
		if(fabs(beta) <= pi_medios){
		  //判断上一次的机器人方向
			if(last_direction == 0){
				ret = 1;
			}else {
				ret = 1;
				//如果相差小于等于５度
				if( fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						iCase = 1;
						ret = -1;
					}else {
						iCase = 2;
					}
				}
			}
		}else{
			
			if(last_direction == 0)//如果是第一次
				ret = -1;
			else {
				ret = -1;
				if(fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						ret = 1;
						iCase = 3;
					}else{
						iCase = 4;
					}
				}
			}
		}
		/*属于哪种情况，相差的角度，与90度的差别，以前的方向，新的方向*/
		ROS_INFO("%s:CalculateDirectionSpeed:\n\
				case %d.\n\
				Beta = %.2lf.\n\
				Diff = %.2lf.\n\
				Last direction = %d,\n\
				new direction = %d",
				sComponentName.c_str(),
				iCase,
				beta*180.0/M_PI,
				(beta - pi_medios)*180.0/M_PI,
				last_direction,
				ret);

		last_direction = ret;
		return ret;
	}

	/*! \fn int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position)
	*	\brief Calcula el sentido de movimiento de una ruta, dependiendo de la posición inicial y el ángulo del robot
	*	\return 1 si el sentido es positivo
	*	\return -1 si el sentido es negativo
	*/
	
	int CalculateDirectionSpeed(geometry_msgs::Pose2D target_position){
		int ret = 1;
		double alpha = pose2d_robot.theta;
		double x =	pose2d_robot.x, y = pose2d_robot.y;
		double ux, uy, vx, vy;
		double beta = 0.0;
		static int last_direction = 0;
		static double pi_medios = M_PI / 2.0, max_diff = 5.0 * M_PI / 180.0;
		int iCase = 0;

		//
		// si la posicion objetivo es la misma del robot, devolvemos 0
		if( (target_position.x == x) && (target_position.y == y) ){
			return 0;
		}
		// Cálculo del vector director del robot respecto al sistema de coordenadas del robot
		ux = cos(alpha);
		uy = sin(alpha);
		// Cálculo del vector entre el punto objetivo y el robot
		vx = target_position.x - x;
		vy = target_position.y - y;

		// Cálculo del ángulo entre el vector director y el vector al punto objetivo
		beta = acos( (ux * vx + uy * vy) / ( sqrt(ux*ux + uy*uy) * sqrt(vx*vx + vy*vy) ) );

		// Devolvemos valor dependiendo del ángulo entre la orientación del robot y la posición objetivo (radianes)
		// Tendremos en cuenta el valor del sentido de avance de la última ruta.
		if(fabs(beta) <= pi_medios){
			// Calculo inicial de direccion
			if(last_direction == 0)
				ret = 1;
			else {
				ret = 1;

				if( fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						iCase = 1;
						ret = -1;

					}else {
						iCase = 2;
					}
				}

			}
		}else{
			// Calculo inicial de direccion
			if(last_direction == 0)
				ret = -1;
			else {
				ret = -1;
				if(fabs(beta - pi_medios) <= max_diff){
					if(last_direction != ret){
						ret = 1;
						iCase = 3;
					}else{
						iCase = 4;
					}
				}

			}
		}

		ROS_INFO("%s:CalculateDirectionSpeed:  case %d. \n\
				Beta = %.2lf. \n\
				Diff = %.2lf. \n\
				Last direction = %d, \n\
				new direction = %d", 
				sComponentName.c_str(),
				iCase, 
				beta*180.0/M_PI, 
				(beta - pi_medios)*180.0/M_PI, 
				last_direction, 
				ret);

		last_direction = ret;
		return ret;
	}

	/*!	\fn ReturnValue Agvs::MergePath()
	 * 	\brief Merges the current path with the next path
	*/
	//将当前路径和下一个路径进行融合
	ReturnValue MergePath(){
		Waypoint new_waypoint, wFirst, wLast;
		Path aux;		//辅助路径
		int direction = 0;

		//从动作服务器获取新点
		if(action_server_goto.isNewGoalAvailable()){
			goto_goal.target = action_server_goto.acceptNewGoal()->target;
			//如果点的数目大于0
			if(goto_goal.target.size() > 0){
				//如果点的数目大于1,则用第二个点得到速度方向
				if(goto_goal.target.size() > 1){
					direction = CalculateDirectionSpeed(goto_goal.target[1].pose);
				}else{//否则使用第一个点计算速度方向
					direction = CalculateDirectionSpeed(goto_goal.target[0].pose);
				}
				//如果速度朝前，那么只是用前面的雷达；如果速度朝后，那么只使用后面的雷达
				if(direction == 1){// Uses only front laser
					SetLaserFront();
				}else{	// Uses only back laser
					SetLaserBack();
				}
				//处理每一个目标点
				for(int i = 0; i < goto_goal.target.size(); i++){
					//打印所有目标点的位置姿态和速度
					//ROS_INFO("%s::MergePath: Point %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", 
																	//sComponentName.c_str(), 
					                                                //i,  
					                                                //goto_goal.target[i].pose.x,
					                                                //goto_goal.target[i].pose.y, 
					                                                //goto_goal.target[i].pose.theta, 
					                                                //goto_goal.target[i].speed);
					//填充新的位点
					new_waypoint.dX = goto_goal.target[i].pose.x;
					new_waypoint.dY = goto_goal.target[i].pose.y;
					new_waypoint.dA = goto_goal.target[i].pose.theta;
					
					//根据计算得的速度方向，应用正或负的速度
					if(direction == 1){
						new_waypoint.dSpeed = fabs(goto_goal.target[i].speed);
					}else{
						new_waypoint.dSpeed = -fabs(goto_goal.target[i].speed);
					}
					ROS_INFO("%s::MergePath: new_waypoint %d (%.3lf, %.3lf, %.3lf) speed = %.3lf", 
																		sComponentName.c_str(), 
																		i,
																		new_waypoint.dX,
																		new_waypoint.dY, 
																		new_waypoint.dA, 
																		new_waypoint.dSpeed);
					//将位点填充到路径中
					pathFilling.AddWaypoint(new_waypoint);
				}
				//根据机器人的转弯半径优化路径
				if(pathFilling.Optimize(AGVS_TURN_RADIUS) != OK)
					ROS_ERROR("%s::GoalCB: Error optimizing the path", sComponentName.c_str());

				//将新的路径添加到队列中
				qPath.push(pathFilling);

				//清除临时的路径对象
				pathFilling.Clear();

				//路径的反馈百分比
				goto_feedback.percent_complete = 0.0;	// Inits the feedback percentage

				//当qPath中有路径时
				if(qPath.size() > 0){
					aux = qPath.front();				    //最前面的路径
					aux.GetWaypoint(0, &wFirst);          //获取路径的第一个位点
					aux.BackWaypoint(&wLast);					//获取最后一个位点
					ROS_INFO("%s::MergePath: Adding new %d points from (%.2lf, %.2lf) to (%.2lf, %.2lf) and ",
							sComponentName.c_str(),
							aux.NumOfWaypoints() ,
							wFirst.dX, wFirst.dY, 
							wLast.dX, wLast.dY);
					ROS_INFO("%s::MergePath: Current number of points = %d ",
							sComponentName.c_str(), 
							pathCurrent.NumOfWaypoints());
					//将队列的第一路径添加到当前的路径中来
					pathCurrent+=qPath.front();
					
					ROS_INFO("%s::MergePath: New number of points = %d",
							sComponentName.c_str(), 
							pathCurrent.NumOfWaypoints());
							
					//丢掉队列最前面的路径
					qPath.pop();
					
					goto_goal.target.clear();	//移除当前的目标
					return OK;
				}
			}
		}
		return ERROR;
	}

	/*! \fn void GoalCB()
		* Called when receiving a new target. (ActionServer)
	*/
	//接受到一个新的目标时会调用这个函数
	void GoalCB()
	{

	}

	/*! \fn void PreemptCB()
		* Called to cancel or replace current mision. (ActionServer)
	*/
	//取消或者代替当前的任务时会执行这个函数
	void PreemptCB()
	{
		bCancel = true;
	}

	/*! \fn void AnalyseCB()
		* Checks the status. (ActionServer)
	*/
	//分析现在的状态
	void AnalyseCB(){
        //如果服务器还是激活的
		if (!action_server_goto.isActive()){
			//ROS_INFO("%s::AnalyseCB: Not active", sComponentName.c_str());
			return;
		}
		//goto_feedback.percent_complete+=1.0;
        //发布反馈信息
		action_server_goto.publishFeedback(goto_feedback);
        
        //如果百分之百完成了
		if(goto_feedback.percent_complete == 100.0){
			//action_server_goto.setSucceeded(goto_result);
			//退出服务
			action_server_goto.setAborted(goto_result);
			ROS_INFO("%s::AnalyseCB: Action finished", sComponentName.c_str());
		}
	}

	/*! \fn void SetLaserFront()
		* Disables laser back, enables laser front
	*/
	bool SetLaserFront(){
		/*s3000_laser::enable_disable srv;

		srv.request.value = false;
		sc_enable_back_laser_.call(srv);
		ROS_INFO("%s::SetLaserFront: Setting laser back to false, ret = %d", sComponentName.c_str(), srv.response.ret);

		srv.request.value = true;
		sc_enable_front_laser_.call(srv);
		ROS_INFO("%s::SetLaserFront: Setting laser front to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
	}

	/*! \fn void SetLaserBack()
		* Disables laser front, enables laser back
	*/
	bool SetLaserBack(){
		/*s3000_laser::enable_disable srv;

		srv.request.value = false;
		sc_enable_front_laser_.call(srv);
		ROS_INFO("%s::SetLaserBack: Setting laser front to false, ret = %d", sComponentName.c_str(), srv.response.ret);

		srv.request.value = true;
		sc_enable_back_laser_.call(srv);
		ROS_INFO("%s::SetLaserBack: Setting laser back to true, ret = %d", sComponentName.c_str(), srv.response.ret);*/
	}
	/*
	 * 激光雷达避障
	 * 距离500mm时,机器人会自动停下,等待障碍物离开
	 * 如果小于0.3m机器人停下
	 * */
	void LaserAvoid(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
		//sensor_msgs::LaserScan * msg = laser_msg;
		
		Avoid = false;
		for(int i = 180 ; i < 900; i += 10){
			if(laser_msg->ranges[i] < 0.5){
				Avoid = true;
				ROS_INFO("danger");
			}
		}
		
	}

};



// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "purepursuit_planner_node");
	
    ros::NodeHandle n;
    purepursuit_planner_node planner(n);

    planner.ControlThread();
    
	
    return (0);
}
// EOF
