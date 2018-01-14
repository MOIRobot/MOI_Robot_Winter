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

using namespace std;

using namespace boost::algorithm;

//将string转换为数字类型
template <class Type>
Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}
void readWaypoints()
{
	vector<float> current_vector;
	vector<string> temp;
	ifstream infile;
	cout << "Open the waypoint.txt......" << endl;
	infile.open("./waypoints.txt");
	string str;
	float result;

	
	//每次读一行
	while(getline(infile, str))
	{
		split(temp, str, is_any_of(";"));
		current_vector.push_back(stringToNum<float>(temp[1]));
		current_vector.push_back(stringToNum<float>(temp[2]));
	}
	infile.close();
	cout << "Close the waypoint.txt......" << endl;
	
	cout << "X" << "          " << "Y" << endl;
	for(vector<float>::iterator it = current_vector.begin(); it != current_vector.end(); it+=2)
		cout << "[" << *it << "," << "  " << *(it+1) << "]" << endl;

}

void readLines()
{
	ifstream infile;
	infile.open("./lines.txt");
	string str;
	
	//save the lines
	vector<string> lines_vector;
	vector<string> temp;
	
	//read a line from infile to s
	while(getline(infile, str)){
		//vector, string, ...
		split(temp, str, is_any_of(";"));
		lines_vector.insert(lines_vector.end(), temp.begin(), temp.end());
	}
	for(vector<string>::iterator it = lines_vector.begin(); it != lines_vector.end(); it++)
		cout << *it << endl;
}
	
int main(int argc, char* argv[])
{

	readWaypoints();
	//readLines();
	
	return 0;
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



