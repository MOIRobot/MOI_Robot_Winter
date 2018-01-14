#include <iostream>
#include <fstream> //for ifstream
#include <string>
#include <sstream> //for stringstream
#include <vector>
#include <ros/ros.h>
#include <cstdio> // for EOF
#include <string.h> 
#include <boost/algorithm/string.hpp>  //for split
#include <cmath> //for sqrt
#include <geometry_msgs/Pose2D.h> 
#include <geometry_msgs/Point.h> 
#include <stack>
#include <nav_msgs/Path.h>


using namespace std;
using namespace boost::algorithm;

//贝塞尔曲线的控制点数
#define BEZIER_CONTROL_POINTS		5
//贝塞尔曲线的最小角度15度
#define MIN_ANGLE_BEZIER			0.261799388

//只需要考虑位置，点的姿态不需要考虑
typedef struct Waypoint{
	int iID;
	float dX;
	float dY;
}Waypoint;

//储存位点
vector<Waypoint> vPoints;

//函数申明
void readWaypoints();
int Optimize(float distance);
float dot2( Waypoint w, Waypoint v);
template<typename Type> Type stringToNum(const string&);
Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t);



int main(int argc, char*argv[])
{
	ros::init(argc, argv, "agv");
	
	ros::NodeHandle node_handle_;
	
	nav_msgs::Path better_path;
	
	ros::Publisher path_pub_;
	
	ros::Rate r(20);
	
	path_pub_ = node_handle_.advertise<nav_msgs::Path>("Path", 10);

	//将waypoints读入vPoints
	readWaypoints();

	Optimize(0.5);
	
	printf("\n------------------Path---------------------\n");
	for(int i = 0; i < vPoints.size(); i++){
		cout << "[" << i << "] = " << vPoints[i].dX << ", " << vPoints[i].dY << endl;
	}
	
	
	//fil path
	int path_size = vPoints.size();
	better_path.poses.resize(path_size);
	for(int i = 0; i < path_size; i++){
		better_path.header.frame_id = "map";
		better_path.poses[i].pose.position.x = vPoints[i].dX;
		better_path.poses[i].pose.position.y = vPoints[i].dY;
	}
	
	while(ros::ok()){
		path_pub_.publish(better_path);
		r.sleep();
	}

	
	return 0;
}

//! Cross product
//.^2
float dot2( Waypoint w, Waypoint v) {
	return (w.dX*v.dX + w.dY*v.dY);
}


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


//将string转换为数字类型
template <typename Type>
Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}
//读入位点
void readWaypoints()
{
	vector<float> float_vector;
	vector<string> string_vector;
	ifstream infile;
	//cout << "Open the waypoint.txt......" << endl;
	infile.open("./waypoints.txt");
	string str;
	float result;

	//填入temp_vector容器
	while(getline(infile, str))
	{	
		//P_0,P_1,P_3...
		if(str == "") //排除多余空行的干扰
			break;
		split(string_vector, str, is_any_of(";"));
		float_vector.push_back(stringToNum<float>(string_vector[1]));//x
		float_vector.push_back(stringToNum<float>(string_vector[2]));//y
	}
	infile.close();
	
	//填入vPoints
	for(int i = 0; i < float_vector.size(); i+=2){
		Waypoint w;
		w.iID = i/2;
		w.dX = float_vector[i]; // x
		w.dY = float_vector[i+1]; // y
		vPoints.push_back(w);
	}
	for(int i = 0; i < vPoints.size(); i++)
		cout <<"P_" << vPoints[i].iID << ":"<< vPoints[i].dX <<", "<< vPoints[i].dY << endl;
	printf("Read %ld Waypoints \n", vPoints.size()) ;

}




#if 1

//! Modifies and adds new waypoints to the route for improving the path
//! \param distance as float, used for the calculation of the new points
//! \return ERROR if Size is lower than 3, distance <= 0 or the waypoints has already been optimized
//! \return OK
/*为了优化路径添加新的位点和修改已有的位点
参数distance用于计算新的点
*/

int Optimize(float distance){
	int i, j=0;
	int a, b, c;
	float mod_ab, mod_bc;
	float dAngle = 0;
	float K= 0.0;
	float Ax = 0.0, Ay = 0.0, Bx = 0.0, By = 0.0, Cx = 0.0, Cy = 0.0;
	float Kt = 1.0 / BEZIER_CONTROL_POINTS;
	Waypoint aux;
	Waypoint ab, bc, ba;
	Waypoint A, B, C;
	vector <Waypoint> new_points;
	
	//判断点容器中点的个数，如果小于2个或者距离为0，则不适合做优化
	if((vPoints.size() < 2) || (distance <= 0.0)){
		return -1;
	}

	//如果容器中的点数为2，那么在两个点中间插入一个点
	if(vPoints.size() == 2){
		aux = vPoints[1];
		vPoints.push_back(aux); 
		vPoints[1].dX = vPoints[0].dX + (aux.dX - vPoints[0].dX) / 2.0; 
		vPoints[1].dY = vPoints[0].dY + (aux.dY - vPoints[0].dY) / 2.0;
	}
	//在新位点的容器中添加两个新的点
	new_points.push_back(vPoints[0]);
	new_points.push_back(vPoints[1]);

/*********************************循环处理位点**********************************************************/
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
	
		/*
		<abc的角度 angle = acos((x1*x2 + y1*y2)/(sqrt(x1^2 + y1^2) * sqrt(x2^2 + y2^2)))
		acos(angle) = (a^2 + b^2 -c^2)/(2 * a * b) 
		*/
		dAngle= acos(dot2(ab,bc)/(mod_ab*mod_bc));

/**插入新的点优化：判断角度大于等于最小的贝塞尔角度****/
		if(fabs(dAngle) >= MIN_ANGLE_BEZIER){
			//删除new_points最后一个点，也就是b点
			new_points.pop_back(); 
/*****如果ab节点之间的距离大于给定的距离，在中间插入新的点*******/
			if(mod_ab > distance){
				//Lo creamos
				ba.dX = -ab.dX;
				ba.dY = -ab.dY;
				K = distance / sqrt(ba.dX * ba.dX + ba.dY * ba.dY);

				aux.dX = Bx + K * ba.dX;	// x = x' + K*Vx
				aux.dY = By + K * ba.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director
				
				new_points.push_back(aux);
			}
			//将b点添加到new_points中
			new_points.push_back(vPoints[b]);
		
/**如果bc节点之间的距离大于给定的距离，在中间插入新的点******/
			if(mod_bc > distance){

				K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
				aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
				aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director

				new_points.push_back(aux);
			}
			//添加位点c
			new_points.push_back(vPoints[c]);
			
		}else{//如果角度小于15度，直接添加位点c
			new_points.push_back(vPoints[c]);
		}
	}//所有位点处理结束

	//清除原来容器中的位点
	vPoints.clear();

/*****第二布优化：贝塞尔曲线优化************************/
	// BEZIER
	vPoints.push_back(new_points[0]);
	vPoints.push_back(new_points[1]);

	//对new_points中的点进行贝塞尔曲线优化
	for(i=2; i < new_points.size(); i++){
	
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
			float t;

			A = new_points[a];
			B = new_points[b];
			C = new_points[c];

			vPoints.pop_back();//删除最后一个点
		
			//用贝塞尔曲线优化路径，优化的点数为5个
			for(int j=1; j <= BEZIER_CONTROL_POINTS; j++) {
				t = (float) j * Kt;
				aux_wp = PosOnQuadraticBezier(A, B, C,  t);
				vPoints.push_back(aux_wp);
			}
		}else{//如果角度小于贝塞尔角度，则直接将位点添加进去
			vPoints.push_back(new_points[c]);
		}
	}

	new_points.clear();

	return 0;
}

#endif


