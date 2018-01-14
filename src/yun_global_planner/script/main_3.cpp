/*
C++读入文本数据的一个例子
stringstream遇到数字就会将它转换为特定的类型
比较适合做数据类型的转换
*/

#include <iostream>
#include <fstream> //for ifstream
#include <string>
#include <sstream> //for stringstream
#include <vector>
#include <cstdio> // for EOF
#include <string.h> 
#include <boost/algorithm/string.hpp>  //for split
#include <cmath> //for sqrt
#include <geometry_msgs/Pose2D.h> 
#include <geometry_msgs/Point.h> 
#include <stack>

#define SIZE 14
#define DEBUG  1

using namespace std;
using namespace boost::algorithm;

//! Data structure for a Waypoint
typedef struct Waypoint{
    //! Id of the magnet
    int iID;
    //! X position
    float dX;
    //! Y position
    float dY;
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

//无向图
#if 0
	float MAP[][SIZE] = {
		{0, 12, DBL_MAX, DBL_MAX, DBL_MAX, 16, 14},
		{12, 0, 10, DBL_MAX, DBL_MAX, 7, DBL_MAX},
		{DBL_MAX, 10, 0, 3, 7, 6, DBL_MAX},
		{DBL_MAX, DBL_MAX, 3, 0, 4, DBL_MAX, DBL_MAX},
		{DBL_MAX, DBL_MAX, 7, 4, 0, 2, 8},
		{16, 7, 6, DBL_MAX, 2, 0, 9},
		{14, DBL_MAX, DBL_MAX, DBL_MAX, 8, 9, 0},
	};
#endif
//float MAP[SIZE][SIZE] = {0}; //动态分配map大小
float **MAP=NULL;
char P[SIZE] = {'A','B','C','D','E','F','G','H','I','J','K', 'L','M', 'N'};

//线段的信息容器
vector<int> line_vector;
vector<Line> LineVector;
vector<Waypoint> WaypointVector;

template<typename Type> Type stringToNum(const string&);
void readline();
void readWaypoints();
float Dist(float x1, float y1, float x2, float y2);
void fillLine(vector<Waypoint> &, vector<int> &);
float DistP2S(geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb);
float Dot2( float x1, float y1, float x2, float y2);
float Dist(float x1, float y1, float x2, float y2);
Waypoint calClosestPoint(Waypoint w1);
void dijkstra(const int numOfNode, //结点的数目
			const int startNode, //起始结点
			float **MAP, //地图数据
			float *distance, //距离
			int *preNode); //路径
void initMap(vector<Waypoint> waypoint_vector, vector<Line> lines);
stack<int> selectPath(int startNode, int goalNode);




//用waypoint数据初始化地图数据
void initMap(vector<Waypoint> waypoint_vector, vector<Line> lines)
{
	int size = waypoint_vector.size();
	
	//动态分配地图内存
	MAP = (float**)malloc(sizeof(float*)*size);
	if(MAP == NULL)
		printf("alloc memory error\n");
	for(int i = 0; i < size; i++)
		*(MAP + i) = (float*)malloc(sizeof(float*) * size);
	
	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){
			if(i == j)
				MAP[i][j] = 0;
			else
				MAP[i][j] = DBL_MAX;
		}
	}
	
	for(vector<Line>::iterator it = lines.begin(); it != lines.end(); it++){
		int row = it->w0.iID;
		int col = it->w1.iID;
		MAP[row][col] = it->length;
		MAP[col][row] = it->length;
	
	}
	
	#if DEBUG
	for(int i = 0; i < size; i++)
		printf("%8d", i);
	cout << endl;
	for(int i = 0; i < size; i++){
		printf("%2d", i);
		for(int j = 0; j < size; j++){
			printf("%8.1f", MAP[i][j]);
		}
		cout << endl;
	}
	#endif
	
	
}


//fillLine(waypoint_vector, line_vector)
//填充Line向量
void fillLine(vector<Waypoint> &waypoint_vector, vector<int> &line_vector)
{	
	Line line;
	for(int i = 0; i < line_vector.size(); i+=2){
		line.id = i/2;
		//一条线段的两个端点
		line.w0 = waypoint_vector[line_vector[i]];
		line.w1 = waypoint_vector[line_vector[i+1]];
		line.length = Dist(line.w0.dX, line.w0.dY, line.w1.dX, line.w1.dY);
		LineVector.push_back(line);
	}
	#if DEBUG
	for(int j = 0 ;j < LineVector.size(); j++){
		cout <<"The LineVector[" << LineVector[j].id <<"]"<< ":" << LineVector[j].length << endl;
	}
	#endif
	printf("Fill %ld Lines finished\n", LineVector.size()) ;
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
	vector<float> waypoint_vector;
	vector<string> temp;
	ifstream infile;
	//cout << "Open the waypoint.txt......" << endl;
	infile.open("./waypoints.txt");
	string str;
	float result;
	
	//每次读一行
	while(getline(infile, str))
	{	
		//P_0,P_1,P_3...
		split(temp, str, is_any_of(";"));
		waypoint_vector.push_back(stringToNum<float>(temp[1]));//x
		waypoint_vector.push_back(stringToNum<float>(temp[2]));//y
	}
	infile.close();
	//cout << "Close the waypoint.txt......" << endl;
	
	//fill waypoint
	for(int i = 0; i < waypoint_vector.size(); i+=2){
		Waypoint w;
		w.iID = i/2;
		w.dX = waypoint_vector[i]; // x
		w.dY = waypoint_vector[i+1]; // y
		WaypointVector.push_back(w);
	}
	//for(int i = 0; i < WaypointVector.size(); i++)
	//cout <<"P_" <<  WaypointVector[i].iID << ":"<< WaypointVector[i].dX <<", "<< WaypointVector[i].dY << endl;
	printf("Read %ld Waypoints \n", WaypointVector.size()) ;

}

//读入线段
void readline()
{
	ifstream infile;
	infile.open("./lines.txt");
	string str;
	
	//定义三个向量，用于保存线的信息
	//vector<int> line_vector;
	vector<string> temp;
	
	//read a line from infile to s
	while(getline(infile, str)){
		//P_0;P_1
		split(temp, str, is_any_of("_;"));
		line_vector.push_back(stringToNum<int>(temp[1]));
		line_vector.push_back(stringToNum<int>(temp[3]));
	}
	//for(int i = 0; i < line_vector.size(); i+=2)
	//cout << line_vector[i] << ", "<< line_vector[i+1] << endl;
	printf("Read %ld Lines \n", line_vector.size()/2) ;
}

//计算两个坐标之间的距离
float Dist(float x1, float y1, float x2, float y2) {
	float diff_x = (x2 - x1);
	float diff_y = (y2 - y1);
	return sqrt( diff_x*diff_x + diff_y*diff_y );
}

//计算交叉点积
float Dot2( float x1, float y1, float x2, float y2) {
	return (x1*x2 + y1*y2); // cross product
}

//计算从当前位置到s0->s1之间的最短距离和s0->s1上对应的点
float DistP2S(geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
{
	float vx,vy, wx, wy;
	float c1, c2, di, b;
	
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

//在waypoint_vector中找到距离w点最近的点
Waypoint calClosestPoint(Waypoint w)
{
	Waypoint closest_point;
	float dis , dis_min = DBL_MAX;
	int id_min = INT_MAX;
	for(int i = 0; i < WaypointVector.size(); i++){
		dis = Dist(WaypointVector[i].dX, WaypointVector[i].dY, w.dX, w.dY);
		if(dis < dis_min){
			dis_min = dis;
			closest_point = WaypointVector[i];
			id_min = i;
		}
	}
	printf("The dis_min point id is : %d\n", id_min);
	return closest_point;
}


void calClosestLine()
{
	geometry_msgs::Pose2D cur_pos;
	Waypoint Pb, Pb_min;
	float dis = 0, dis_min = 100;
	for(int i = 0; i < LineVector.size(); i++){
		dis = DistP2S(cur_pos, LineVector[i].w0, LineVector[i].w1, &Pb);
		if(dis_min > dis){
			dis_min = dis;
			Pb_min = Pb;
		}
	}
	cout << "The dis_min is :" << dis_min << endl;
	cout << "The Pb_min is :" << Pb_min.dX << "," << Pb_min.dY << endl;

}

void dijkstra(
			const int numOfNode, //结点的数目
			const int startNode, //起始结点
			float **MAP, //地图数据
			float *distance, //起始点到哥哥结点的距离
			int *preNode) //路径
{
	//是否在S集合
	vector<bool> isInS;
	isInS.assign(numOfNode, false);
	
	//初始化结点
	for(int i = 0 ; i < numOfNode; i++){
	
		//起始结点到各个结点的距离
		distance[i] = MAP[startNode][i];
		//前一个结点
		if(distance[i] < DBL_MAX)
			preNode[i] = startNode;
		else
			preNode[i] = -1;
	}
	
	preNode[startNode] = -1;
	
	//将头一个节点放入S集合
	isInS[startNode] = true;
	
	//处理结点
	int nextNode = startNode;
	
	//处理U集合中的每一个顶点
	for(int i  = 1; i < numOfNode; i++){
		
		//下一次寻找的起始结点
		int nextNode = startNode;
		float miniDistance = DBL_MAX;
		for(int j = 0; j < numOfNode; j++){
			if((isInS[j] == false) && (distance[j] < miniDistance)){
				miniDistance  = distance[j];
				nextNode = j;
			}
		}
		
		//放入S集合
		isInS[nextNode] = true;
		
		//更新distance
		for(int j = 0; j < numOfNode; j++){
			
			if((isInS[j] == false) && (MAP[nextNode][j] < DBL_MAX)){
				float temp = (distance[nextNode] + MAP[nextNode][j]);
				if(temp < distance[j]){
					distance[j] = temp;
					preNode[j] = nextNode;
				}
 			}
		}
	}
}

//使用dijkstra算法计算最优路径
stack<int> selectPath(int startNode, int goalNode)
{
	float distance[SIZE]= {0};
	int preNode[SIZE] ={0};
	stack<int> trace;
	int index = goalNode;
	dijkstra(SIZE, startNode, MAP, distance, preNode);
	while(preNode[index] != -1){
		trace.push(preNode[index]);
		index = preNode[index];
	}
	#if DEBUG
	
	cout << "trace:";
	while(!trace.empty()){
		cout << trace.top() << "--" ;
		trace.pop();
	}
	cout <<goalNode; 
	cout << " distance: " << distance[goalNode] << endl;
	
	#endif
	
	return trace;
}

/*优化路径
由于不知道机器人在每个位点处的角度方向
所以需要用贝尔塞尔曲线优化路径
*/
void Optimize(float distance)
{



}

Waypoint PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t)
{
	Waypoint aux;

	//B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
	//二次贝塞尔曲线方程
	//Bx(t)
	aux.dX = (1.0-t)*(1.0-t)*cp0.dX + 2*t*(1.0-t)*cp1.dX + t*t*cp2.dX;
	//By(t)
	aux.dY = (1.0-t)*(1.0-t)*cp0.dY + 2*t*(1.0-t)*cp1.dY + t*t*cp2.dY;

	return aux;
}

int main(int argc, char* argv[])
{
	
	readWaypoints();
	readline();
	fillLine(WaypointVector, line_vector);
	initMap(WaypointVector, LineVector);
	
	int startNode, goalNode;
	if(argc < 3){
		printf("The argc is less than 3\n");
		return -1;
	}
	startNode = atoi(argv[1]);
	goalNode = atoi(argv[2]);
	
	selectPath(startNode, goalNode);
	
/*
	int startNode = 0; // startNode
	float distance[SIZE];
	int preNode[SIZE];
	//char P[SIZE] = {'A','B','C','D','E','F','G'};
	
	dijkstra(SIZE, startNode, MAP, distance, preNode);
	for(int j = 0; j < SIZE; j++){ //goalNode
		int index = j;
		stack<int> trace;
		while(preNode[index] != -1){
			trace.push(preNode[index]);
			index = preNode[index];
		}
	
		cout << "trace:";
		while(!trace.empty()){
			cout << P[trace.top()] << "--" ;
			trace.pop();
		}
		cout <<P[j]; 
		cout << " distance: " << distance[j] << endl;
	}
	*/
	return 0;
}

/*
dijkstra算法是思路：
	引进了两个集合，S和U，S的作用是记录已求出最短路径的顶点（以及相应的最短路径长度），
而U中则是记录还未求出最短路径的顶点（以及该顶点到起点s的距离）。
	1、初始化，S中只包含s；U中包含s以外的其他顶点，且U中顶点的距离为“起点到该顶点的距离”
例如，U中额顶点v的距离为（s，v）的长度，然后s和v不相邻，则v的距离为DBL_MAX。
	2、从U中选取“距离最短的顶点k”，并将顶点k加入S中，同时，从U中移除顶点k。
	3、更新U中各个顶点到s的距离，之所以要跟新U中顶点的距离，是由于上一布中确定了k是求出最短路径
的顶点，从而可以利用k来更新其他顶点的距离，例如，（s,v）的距离可能大于(s,k)+(k,v)的距离。
	4、重复步骤2,3,指导遍历完所有的顶点。

*/





















