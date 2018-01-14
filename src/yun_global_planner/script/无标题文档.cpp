
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
