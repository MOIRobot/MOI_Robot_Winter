#include <iostream>
#include <vector>
#include <stack>
#include <limits.h> //for INT_MAX

#define SIZE 7

using namespace std;

//无向图
int map_2[][SIZE] = {
	{0, 12, INT_MAX, INT_MAX, INT_MAX, 16, 14},
	{12, 0, 10, INT_MAX, INT_MAX, 7, INT_MAX},
	{INT_MAX, 10, 0, 3, 7, 6, INT_MAX},
	{INT_MAX, INT_MAX, 3, 0, 4, INT_MAX, INT_MAX},
	{INT_MAX, INT_MAX, 7, 4, 0, 2, 8},
	{16, 7, 6, INT_MAX, 2, 0, 9},
	{14, INT_MAX, INT_MAX, INT_MAX, 8, 9, 0},
};

void dijkstra(
			const int numOfVertex,//顶点数
			const int startVectex, //源节点
			int (map)[][SIZE], //地图
			int *distance, ///各个节点到达源节点的距离
			int *preVertex) //各个节点的前一个节点
{
	vector<bool> isInS; //是否在S集合
	isInS.reserve(0);
	isInS.assign(numOfVertex, false); //初始化，所有的节点都不在S集合中
	
	//初始化distance
	for(int i = 0; i < numOfVertex; i++){
	
		//起始点到到各个顶点的距离
		distance[i] = map[startVectex][i];
		
		//如果起始点到各个定点的距离小于最大值
		if(distance[i] < INT_MAX)
			preVertex[i] = startVectex;
		else
			//不知道前一个节点是什么
			preVertex[i] = -1;
	}
	
	preVertex[startVectex] = -1;
	
	isInS[startVectex] = true;   
	
	//u代表者正在操作的顶点
	int u = startVectex;
	
	/*这里循环从1开始是因为开始节点已经存放在S中了，
		还有numOfVertex-1个节点要处理*/
	for(int i = 1; i < numOfVertex; i++){
	
		//选择distance最小的节点作为下一个节点
		int nextVertex = u;
		int tempDistance = INT_MAX;
		for(int j = 0; j < numOfVertex; j++){
		
			//寻找不在S集合中并且是最小距离的节点
			if((isInS[j] == false) && (distance[j]) < tempDistance)
			{
				nextVertex = j;
				tempDistance = distance[j];
			}
		}
		
		//放入S集合
		isInS[nextVertex] = true; 
		
		//下一次寻找的起始点
		u = nextVertex; 
		
		//更新distance
		for(int j = 0; j < numOfVertex; j++){
		
			//不在S集合中并且当前点和各个节点是相邻的
			if((isInS[j] == false) && (map[u][j] < INT_MAX)){
			
				/*startVectex 到u节点的距离加上u到各个节点的距离
				小于startVectex 到u节点的距离*/
				int temp = distance[u] + map[u][j];
				if(temp < distance[j]){
					distance[j] = temp;
					preVertex[j] = u;
				}
			}
		
		}
	
	}

}


int main(int argc, char*argv[])
{
	int distance[SIZE];
	int preVertex[SIZE];
	char P[SIZE] = {'A','B','C','D','E','F','G'};
	
	for(int i = 0; i < 1; i++){//起始点startNode
	
		dijkstra(SIZE, i, map_2, distance, preVertex);  
		for(int j =0; j < SIZE; ++j)//终点goalNode
		{
			int index = j;  
			stack<int > trace;
			
			//从终点开始往回找
			while (preVertex[index] != -1) {
				trace.push(preVertex[index]);
				index = preVertex[index];
			}
			cout << "路径：";  
			while (!trace.empty()) {
				cout<<P[trace.top()]<<" -- ";  
				trace.pop();  
			}
			cout <<P[j];  
			cout <<" 距离是："<<distance[j]<<endl; 
		}
	}
	return 0;
}









