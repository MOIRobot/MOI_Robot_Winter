#include <pluginlib/class_list_macros.h>
#include "yun_global_planner/yun_global_planner.h"

#define DEBUG true

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(yun_global_planner::YunGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace boost::algorithm;

//Default Constructor
namespace yun_global_planner {

	YunGlobalPlanner::YunGlobalPlanner (){
	}

	YunGlobalPlanner::YunGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	{
		initialize(name, costmap_ros);
	}

	void YunGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		
		ROS_INFO("YunGlobalPlanner initialize\n");
		
		isOptimize = false;
		
		//发布路径
		path_pub_ = node_handle_.advertise<nav_msgs::Path>("Path", 1);
		
		//后面改成用rospkg.RosPack() #rospath，相对路径
		pointPath = "/home/jobsg/MOI_Robot_Winter/src/yun_global_planner/script/waypoints.txt";
		linePath = "/home/jobsg/MOI_Robot_Winter/src/yun_global_planner/script/lines.txt";
		
		//读取位点
		readWaypoints();
		
		//读取线段数据
		readLines();
		
		//填充LineVector向量
		fillLine(vPoints, pointPairs);
		
		cout << " LineVector size: " << LineVector.size() << "   vPoints size: " << vPoints.size() << endl;
	
	}

	bool YunGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
									const geometry_msgs::PoseStamped& goal,  
									std::vector<geometry_msgs::PoseStamped>& plan)
	{
		//fill the waypoint
		//添加4个点，线路上最靠近起始点和终点的两个点；6条线段：两个最靠近点和两个最靠近位点的线段。
		Waypoint startNode, goalNode, closest2startNode, closest2goalNode;
		Line line_start, line_goal, line_temp; 
		int line_id1, line_id2;
		
		/*************add four node to vPoints*************************/
		startNode.iID = vPoints.size();
		startNode.dX = start.pose.position.x;
		startNode.dY = start.pose.position.y;
		vPoints.push_back(startNode);
		
		goalNode.iID = vPoints.size();
		goalNode.dX = goal.pose.position.x;
		goalNode.dY = goal.pose.position.y;
		vPoints.push_back(goalNode);
		
		//find the closest point in lines
		closest2startNode = calClosestPoint(startNode, &line_start.length, &line_id1);
		closest2goalNode  = calClosestPoint(goalNode, &line_goal.length , &line_id2);
		
		closest2startNode.iID = vPoints.size();
		vPoints.push_back(closest2startNode);
		
		closest2goalNode.iID = vPoints.size();
		vPoints.push_back(closest2goalNode);
		
		
		/****************add 6 lines to LineVector*******************/
		//如果closest2startNode和closest2goalNode在同一条line上
		if(line_id1 == line_id2){
			line_temp.id = LineVector.size();
			line_temp.w0 = startNode;
			line_temp.w1 = goalNode;
			line_temp.length = Dist(line_temp.w0.dX, line_temp.w0.dY, line_temp.w1.dX, line_temp.w1.dY);
			LineVector.push_back(line_temp);
		}else{
			//fill start_line and goal line
			line_start.id = LineVector.size();
			line_start.w0 = startNode;
			line_start.w1 = closest2startNode;
			LineVector.push_back(line_start);
			
			line_goal.id = LineVector.size();
			line_goal.w0 = goalNode;
			line_goal.w1 = closest2goalNode;
			LineVector.push_back(line_goal);
			
			line_temp.id = LineVector.size();
			line_temp.w0 = LineVector[line_id1].w0;
			line_temp.w1 = closest2startNode;
			line_temp.length = Dist(line_temp.w0.dX, line_temp.w0.dY, line_temp.w1.dX, line_temp.w1.dY);
			LineVector.push_back(line_temp);
			
			line_temp.id = LineVector.size();
			line_temp.w0 = LineVector[line_id1].w1;
			line_temp.w1 = closest2startNode;
			line_temp.length = Dist(line_temp.w0.dX, line_temp.w0.dY, line_temp.w1.dX, line_temp.w1.dY);
			LineVector.push_back(line_temp);
			
			line_temp.id = LineVector.size();
			line_temp.w0 = LineVector[line_id2].w0;
			line_temp.w1 = closest2goalNode;
			line_temp.length = Dist(line_temp.w0.dX, line_temp.w0.dY, line_temp.w1.dX, line_temp.w1.dY);
			LineVector.push_back(line_temp);
			
			line_temp.id = LineVector.size();
			line_temp.w0 = LineVector[line_id2].w1;
			line_temp.w1 = closest2goalNode;
			line_temp.length = Dist(line_temp.w0.dX, line_temp.w0.dY, line_temp.w1.dX, line_temp.w1.dY);
			LineVector.push_back(line_temp);
			
		}
		//initMap
		initMap(vPoints, LineVector);
		
		//如果是在同一条线路上
		/*if(line_id1 == line_id2){
			//选择最优路径
			selectPath(startNode.iID, goalNode.iID, vPoints.size());
		}else
		*/
		{
			selectPath(closest2startNode.iID, goalNode.iID, vPoints.size());
		}
		
		/*
		//vPoints里面的位点ID和索引号大小是一样的
		for(int i = 0; i < vPoints.size(); i++)
			cout <<"P_" << vPoints[i].iID << ":"<< vPoints[i].dX <<", "<< vPoints[i].dY << endl;
		*/
		
		//Optimize path
		Optimize(AGV_TURN_RADIUS);
		
		//重新分配gui_path的大小
		gui_path.poses.resize(trace.size());
		gui_path.header.frame_id = "map";
		
		
		plan.push_back(start);
		
		//在这里位点的ID和索引是否是一样的？
		for(int i = 0; i < trace.size(); i++)
		{
			geometry_msgs::PoseStamped new_goal = goal;
			tf::Quaternion goal_quat = tf::createQuaternionFromYaw(0);
			new_goal.pose.position.x = trace[i].dX;
			new_goal.pose.position.y = trace[i].dY;
			new_goal.pose.orientation.x = goal_quat.x();
			new_goal.pose.orientation.y = goal_quat.y();
			new_goal.pose.orientation.z = goal_quat.z();
			new_goal.pose.orientation.w = goal_quat.w();
			plan.push_back(new_goal);
			
			gui_path.poses[i].pose.position.x = trace[i].dX;
			gui_path.poses[i].pose.position.y = trace[i].dY;
			
		}
		//填充路径
		plan.push_back(goal);
		
		//发布路径，用于显示
		path_pub_.publish(gui_path);
		
		vPoints.pop_back();
		vPoints.pop_back();
		vPoints.pop_back();
		vPoints.pop_back();
		if(line_id1 == line_id2)
			LineVector.pop_back();
		else{
			LineVector.pop_back();
			LineVector.pop_back();
			LineVector.pop_back();
			LineVector.pop_back();
			LineVector.pop_back();
			LineVector.pop_back();
		}
		trace.clear();
		
		
		
		return true;
	}
	
	/*从硬盘中读取点的数据*/
	void YunGlobalPlanner::readWaypoints()
	{
		vector<float> float_vector;
		vector<string> string_vector;
		ifstream infile;
		//cout << "Open the waypoint.txt......" << endl;
		infile.open(pointPath);
		string str;
		float result;

		//填入string_vector容器
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
		#if DEBUG
		
		for(int i = 0; i < vPoints.size(); i++)
			cout <<"P_" << vPoints[i].iID << ":"<< vPoints[i].dX <<", "<< vPoints[i].dY << endl;
		printf("Read %ld Waypoints \n", vPoints.size()) ;
		
		#endif

	}
	
	//将string转换为数字类型
	template <typename Type>
	Type YunGlobalPlanner::stringToNum(const string& str)
	{
		istringstream iss(str);
		Type num;
		iss >> num;
		return num;
	}
	
	//获取二次贝塞尔曲线，三个控制点，t位控制参数取[0-1]之间
	Waypoint YunGlobalPlanner::PosOnQuadraticBezier(Waypoint cp0, Waypoint cp1, Waypoint cp2, float t){
		Waypoint aux;
		
		//B(t)= (1-t)^2 * cp0 + 2*t*(1-t)*cp1 + t^2 * cp2;
		//二次贝塞尔曲线方程
		//Bx(t)
		aux.dX = (1.0-t)*(1.0-t)*cp0.dX + 2*t*(1.0-t)*cp1.dX + t*t*cp2.dX;
		//By(t)
		aux.dY = (1.0-t)*(1.0-t)*cp0.dY + 2*t*(1.0-t)*cp1.dY + t*t*cp2.dY;
		
		return aux;
	}
	
	//! Cross product, .^2
	float YunGlobalPlanner::dot2( Waypoint w, Waypoint v) {
		return (w.dX*v.dX + w.dY*v.dY);
	}
	
	int YunGlobalPlanner::Optimize(float distance){
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
		if((trace.size() < 2) || (distance <= 0.0)){
			return -1;
		}

		//如果容器中的点数为2，那么在两个点中间插入一个点
		if(trace.size() == 2){
			aux = trace[1];
			trace.push_back(aux); 
			trace[1].dX = trace[0].dX + (aux.dX - trace[0].dX) / 2.0; 
			trace[1].dY = trace[0].dY + (aux.dY - trace[0].dY) / 2.0;
		}
		//在新位点的容器中添加两个新的点
		new_points.push_back(trace[0]);
		new_points.push_back(trace[1]);

		/*******************循环处理位点****************/
		for(i=2; i < trace.size(); i++){
			//三个控制点
			a = i-2;
			b = i-1;
			c = i;
		
			//位点的距离a,b,c的x,y坐标
			Ax = trace[a].dX;
			Ay = trace[a].dY;
			Bx = trace[b].dX;
			By = trace[b].dY;
			Cx = trace[c].dX;
			Cy = trace[c].dY;

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
				new_points.push_back(trace[b]);
			
				/**如果bc节点之间的距离大于给定的距离，在中间插入新的点******/
				if(mod_bc > distance){

					K = distance / sqrt(bc.dX * bc.dX + bc.dY * bc.dY);
					aux.dX = Bx + K * bc.dX;	// x = x' + K*Vx
					aux.dY = By + K * bc.dY;	// y = y' + K*Vy	//(Vx, Vy) vector director

					new_points.push_back(aux);
				}
				//添加位点c
				new_points.push_back(trace[c]);
				
			}else{//如果角度小于15度，直接添加位点c
				new_points.push_back(trace[c]);
			}
		}//所有位点处理结束

		//清除原来容器中的位点
		trace.clear();

		/*****第二布优化：贝塞尔曲线优化**************/
		// BEZIER
		trace.push_back(new_points[0]);
		trace.push_back(new_points[1]);

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

				trace.pop_back();//删除最后一个点
			
				//用贝塞尔曲线优化路径，优化的点数为5个
				for(int j=1; j <= BEZIER_CONTROL_POINTS; j++) {
					t = (float) j * Kt;
					aux_wp = PosOnQuadraticBezier(A, B, C,  t);
					trace.push_back(aux_wp);
				}
			}else{//如果角度小于贝塞尔角度，则直接将位点添加进去
				trace.push_back(new_points[c]);
			}
		}
		//修改ID
		for(int i = 0 ; i < trace.size(); i++){
			trace[i].iID = i;
		}
		
		new_points.clear();
		
		
		//ROS_INFO("Path Optimize finished!\n ");
		
		return 0;
	}
	
	//读入线段
	void YunGlobalPlanner::readLines()
	{
		ifstream infile;
		infile.open(linePath);
		string str;
		vector<string> temp;
		
		//read a line from infile to s
		while(getline(infile, str)){
			//P_0;P_1
			split(temp, str, is_any_of("_;"));
			pointPairs.push_back(stringToNum<int>(temp[1]));
			pointPairs.push_back(stringToNum<int>(temp[3]));
		}
		//for(int i = 0; i < pointPairs.size(); i+=2)
			//cout << pointPairs[i] << ", "<< pointPairs[i+1] << endl;
		printf("Read %ld Lines \n", pointPairs.size()/2) ;
	}
	
	
	//计算两个坐标之间的距离
	float YunGlobalPlanner::Dist(float x1, float y1, float x2, float y2) {
		float diff_x = (x2 - x1);
		float diff_y = (y2 - y1);
		return sqrt( diff_x*diff_x + diff_y*diff_y );
	}
	
	//计算交叉点积
	float YunGlobalPlanner::Dot2( float x1, float y1, float x2, float y2) {
		return (x1*x2 + y1*y2); // cross product
	}
	
	
	//填充Line向量
	void YunGlobalPlanner::fillLine(vector<Waypoint> &waypoint_vector, vector<int> &pointPairs)
	{
		Line line;
		
		for(int i = 0; i < pointPairs.size(); i+=2){
			line.id = i/2;
			//一条线段的两个端点
			line.w0 = waypoint_vector[pointPairs[i]];
			line.w1 = waypoint_vector[pointPairs[i+1]];
			line.length = Dist(line.w0.dX, line.w0.dY, line.w1.dX, line.w1.dY);
			LineVector.push_back(line);
			
		}
		#if DEBUG
		for(int j = 0 ;j < LineVector.size(); j++){
			cout <<"Distance of LineVector[" << LineVector[j].id <<"]"<< ":" << LineVector[j].length << endl;
		}
		
		printf("LineVector size %ld \n", LineVector.size());
		#endif
	}
	
	//dijkstra算法
	void YunGlobalPlanner::dijkstra(
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
	void YunGlobalPlanner::selectPath(int startNode, int goalNode, int size)
	{
		float distance[size];
		int preNode[size];
		int index = goalNode;
		
		dijkstra(size, startNode, MAP, distance, preNode);
		
		trace.push_back(vPoints[goalNode]);
		while(preNode[index] != -1){
			trace.push_back(vPoints[preNode[index]]);
			index = preNode[index];
		}
		//反转元素位置
		reverse(trace.begin(), trace.end());
		
		#if 0
		
		cout << "trace:";
		for(int i = 0; i < trace.size(); i++){
			cout << trace[i].iID << "--" ;
		}
		cout << goalNode; 
		cout << " distance: " << distance[goalNode] << endl;
		
		#endif
		
	}
	
	
	//计算从当前位置到s0->s1之间的最短距离和s0->s1上对应的点
	float YunGlobalPlanner::DistP2S(geometry_msgs::Pose2D current_position, Waypoint s0, Waypoint s1, Waypoint *Pb)
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
			Pb->dX = s1.dX;
			Pb->dY = s1.dY;
			return di;
		}

		b = c1 / c2;
		Pb->dX = s0.dX + b * vx;
		Pb->dY = s0.dY + b * vy;

		di = Dist(current_position.x, current_position.y, Pb->dX, Pb->dY);

		return di;
	
	}
	
	//计算在所有线段上距离当前位点最近的一个点，并通过指针返回最短的距离和最近点所在线段的id
	Waypoint YunGlobalPlanner::calClosestPoint(Waypoint w, float* length, int* line_id)
	{
		geometry_msgs::Pose2D cur_pos;
		Waypoint Pb, Pb_min;
		float dis = 0, dis_min = DBL_MAX;
		
		cur_pos.x = w.dX;
		cur_pos.y = w.dY;
		
		for(int i = 0; i < LineVector.size(); i++){
			dis = DistP2S(cur_pos, LineVector[i].w0, LineVector[i].w1, &Pb);
			if(dis_min > dis){
				dis_min = dis;
				Pb_min = Pb;
				*line_id = i;
			}
		}  
		
		*length = dis_min;
		
		#if 0
		cout << "The dis_min is :" << dis_min << endl;
		cout << "The Pb_min is :" << Pb_min.dX << "," << Pb_min.dY << endl;
		#endif
		
		return Pb_min;

	}
	
	//用waypoint数据初始化地图数据
	void YunGlobalPlanner::initMap(vector<Waypoint> waypoint_vector, vector<Line> lines)
	{
		int size = waypoint_vector.size();
		int row, col;
		
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
		
		//根据每个点的索引号排列成位点矩阵
		for(vector<Line>::iterator it = lines.begin(); it != lines.end(); it++)
		{
			row = it->w0.iID;
			col = it->w1.iID;
			MAP[row][col] = it->length;
			MAP[col][row] = it->length;
		}
	
		#if 0
		for(int i = 0; i < size; i++)
			printf("%4d", i);
		cout << endl;
		for(int i = 0; i < size; i++){
			printf("%2d", i);
			for(int j = 0; j < size; j++){
				printf("%4.1f", MAP[i][j]);
			}
			cout << endl;
		} 
		#endif
	
	
	}
	
};




















