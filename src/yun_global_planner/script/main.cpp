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
#include <cmath>

using namespace std;
using namespace boost::algorithm;

//用于储存位点的信息
typedef struct Waypoint{
	int id;
	float x;
	float y;
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

//线段的信息容器
vector<int> line_vector;
vector<Line> LineVector;
vector<Waypoint> WaypointVector;

template<typename Type> Type stringToNum(const string&);
void readline();
void readline_2();
void readWaypoints();
void readWaypointsBack();
float pointsDistance(Waypoint , Waypoint );
void fillLine(vector<Waypoint> &, vector<int> &);



//fillLine(waypoint_vector, line_vector)
void fillLine(vector<Waypoint> &waypoint_vector, vector<int> &line_vector)
{	
	Line line;
	for(int i = 0; i < line_vector.size(); i+=2){
		line.id = i/2;
		//一条线段的两个端点
		line.w0 = waypoint_vector[line_vector[i]];
		line.w1 = waypoint_vector[line_vector[i+1]];
		line.length = pointsDistance(line.w0, line.w1);
		LineVector.push_back(line);
	}
	for(int j = 0 ;j < LineVector.size(); j++){
		cout <<"The LineVector[" << LineVector[j].id <<"]"<< ":" << LineVector[j].length << endl;
	}
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

void readWaypoints()
{
	vector<float> waypoint_vector;
	vector<string> temp;
	ifstream infile;
	cout << "Open the waypoint.txt......" << endl;
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
	cout << "Close the waypoint.txt......" << endl;
	
	//fill waypoint
	cout << "X" << "          " << "Y" << endl;
	for(int i = 0; i < waypoint_vector.size(); i+=2){
		//cout << "[" << waypoint_vector[i] << "," << "  " << waypoint_vector[i+1] << "]" << endl;
		//fill waypoint_vector
		Waypoint w;
		w.id = i/2;
		w.x = waypoint_vector[i]; // x
		w.y = waypoint_vector[i+1]; // y
		WaypointVector.push_back(w);
	}
	for(int i = 0; i < WaypointVector.size(); i++)
		cout <<"P_" <<  WaypointVector[i].id << ":"<< WaypointVector[i].x <<", "<< WaypointVector[i].y << endl;
	
}

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
	for(int i = 0; i < line_vector.size(); i+=2)
		cout << line_vector[i] << ", "<< line_vector[i+1] << endl;
}
void readline_2()
{
	ifstream infile;
	infile.open("./line.txt");
	string str;
	
	//save the line
	vector<string> line_vector;
	vector<string> temp;
	
	//read a line from infile to s
	while(getline(infile, str)){
		//vector, string, ...
		split(temp, str, is_any_of(";"));
		line_vector.insert(line_vector.end(), temp.begin(), temp.end());
	}
	for(vector<string>::iterator it = line_vector.begin(); it != line_vector.end(); it++)
		cout << *it << endl;
}

float pointsDistance(Waypoint w1, Waypoint w2){
	return sqrt((w1.x - w2.x)*(w1.x - w2.x) + (w1.y - w2.y)*(w1.y - w2.y));
}


void readWaypointsBack()
{
	vector<float> current_vector;
	
	ifstream infile;
	cout << "Open the waypoint.txt......" << endl;
	infile.open("./waypoints.txt");
	string s;
	float result;
	
	//每次读一行
	while(getline(infile, s))
	{
		int i = 0;
		
		/*定义标准的字符流  stringstream通常是用来做数据转换的。float 与 string的转换  */
		stringstream input_ss(s);
		while (!!input_ss && !input_ss.eof())
		{
			//printf("i = %d-- the char is %c\n",i, input_ss.peek());
			
			//peek()返回并判断当前指针指向的字符，其功能是从输入流中读取一个字符 但该字符并未从输入流中删除
			switch(input_ss.peek())
			{
			case EOF:
				break;
			case ';':
				//读入一个字符，并把指针指向下一个字符
				input_ss.get();
				break;
				
			default:
				//跳过前面的三个字符
				if(i>=3)
				{
					float value;
					//字符转换为浮点数
					input_ss >> value;
					current_vector.push_back(value);
					//printf("Get a float %f\n", value);
				}else{
					input_ss.get();
				}
				break;
			}
			i++;
		
		}
		
	}
	infile.close();
	cout << "Close the waypoint.txt......" << endl;
	
	//打印读入的转换后的浮点数
	for(int i = 0; i < current_vector.size(); i++)
		printf("current_vector [%d] = %f\n", i, current_vector[i]);

}

int main(int argc, char* argv[])
{
/*
	Waypoint w1, w2;
	w1.ID = 1;
	w1.x = 0.0;
	w1.y = 0.0;
	w2.ID = 2;
	w2.x = 3.0;
	w2.y = 4.0;
	cout << "The distance between w1 and w2 is :" << pointsDistance(w1, w2) << endl;
*/
	readWaypoints();
	readline();
	fillLine(WaypointVector, line_vector);
	
	
	
	return 0;
}










