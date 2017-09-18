#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from math import radians, copysign, sqrt, pow, pi,cos
from nav_msgs.msg import Path
import PyKDL
import tf
import math

#求两个点之间点斜率　传入参数　坐标点　ｘ　ｙ
def Slope(GX,CX,GY,CY):
	if (GX-CX)==0.0:
		if GY>CY:
			return 10000000.0
		else:
			return -10000000.0
	else:
		return float(GY-CY)/(GX-CX)
#求两个点之间点斜率　传入参数　坐标点　ｘ　ｙ
def SlopePose(pose1,pose2):
	GX=pose1.pose.position.x
	GY=pose1.pose.position.y
	CX=pose2.pose.position.x
	CY=pose2.pose.position.y
	
	if (GX-CX)==0.0:
		if GY>CY:
			return 10000000.0
		else:
			return -10000000.0
	else:
		return float(GY-CY)/(GX-CX)
#求两条直线的交点　传入参数　第一条直线的两个点　第二条直线的两个点
def CrossPoint(pose1,pose2,pose3,pose4):
	x1=pose1.pose.position.x
	y1=pose1.pose.position.y
	x2=pose2.pose.position.x
	y2=pose2.pose.position.y
	
	x3=pose3.pose.position.x
	y3=pose3.pose.position.y
	x4=pose4.pose.position.x
	y4=pose4.pose.position.y
	if (x2==x1):
		k2=float(y4-y3)/(x4-x3)
		c2=y3-k2*x3
		#y=k2*x+c2
		Y=k2*x1+c2
		return x1,Y
	if (x3==x4):
		k1=float(y2-y1)/(x2-x1)
		c1=y1-k1*x1
		Y=k1*x3+c1
		return x3,Y
	k2=float(y4-y3)/(x4-x3)
	c2=y3-k2*x3
	k1=float(y2-y1)/(x2-x1)
	c1=y1-k1*x1
	X=float(c1-c2)/(k2-k1)
	Y=float(k1*c2-c1*k2)/(k1-k2)
	return X,Y


# 计算目标点与当前点的方向　坐标
def canculateAngle(GX,GY,CX,CY):
	ERR_X=GX-CX
	ERR_Y=GY-CY
	if ERR_X>0:
		return math.atan(ERR_Y/ERR_X)
	elif ERR_X<0:
		if ERR_Y>0:
			return math.atan(ERR_Y/ERR_X)+pi
		else:
			return math.atan(ERR_Y/ERR_X)-pi
	else:
		if ERR_Y>0:
			return pi/2
		else:
			return 0-pi/2
# 计算目标点与当前点的方向　传入参数目标点ｇｏａｌｐｏｓｅ　当前点　ｃｕｒｒｅｎｔｐｏｓｅ
def canculate_G_C_Angle(gPose,cPose):
	ERR_X=gPose.pose.position.x-cPose.pose.position.x
	ERR_Y=gPose.pose.position.y-cPose.pose.position.y
	if ERR_X>0:
		return math.atan(ERR_Y/ERR_X)
	elif ERR_X<0:
		if ERR_Y>0:
			return math.atan(ERR_Y/ERR_X)+pi
		else:
			return math.atan(ERR_Y/ERR_X)-pi
	else:
		if ERR_Y>0:
			return pi/2
		else:
			return 0-pi/2
#计算两个点中间的距离
def canculateDistance(pose1,pose2):
	return sqrt(pow((pose1.pose.position.x-pose2.pose.position.x), 2)+pow(pose1.pose.position.y-pose2.pose.position.y,2))
	
#将四元数转化为角度信息	
def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

#从A*算法得出的路径中再次挑选新的目标点　减少Ａ*的点数
def newPathFromAStar(path):
	i=15
	#每１米最少一个目标点　防止长距离角度过小的碰撞
	poses=[]
	length=len(path.poses)	
	
	if length>15:
		lastj=0
		lastGD=quat_to_angle(path.poses[15].pose.orientation)
		poses.append(path.poses[15])

	while (i<length-20) and (length>15):
		GD=quat_to_angle(path.poses[i].pose.orientation)
		errDirection=GD-lastGD
		if(errDirection>3.14):
			errDirection=2*3.1415-errDirection
		elif(errDirection<-3.14):
			errDirection=2*3.1415+errDirection
		
		#0.175 10du 0.35 20 0.524 30degree
		
		#遇到拐角的地方　向外进行扩展目标点　根据斜率进行扩展
		if(abs(errDirection))>0.35:
			#向外部扩展目标点
			x=path.poses[i].pose.position.x
			y=path.poses[i].pose.position.y
			x1=path.poses[i+10].pose.position.x
			y1=path.poses[i+10].pose.position.y
			x2=path.poses[i-10].pose.position.x
			y2=path.poses[i-10].pose.position.y
			k1=Slope(x1,x,y1,y)
			k2=Slope(x,x2,y,y2)
			if y1>y2:
				if x1>x2:
					if k1>k2:
						path.poses[i].pose.position.x=x1
						path.poses[i].pose.position.y=y2
					else:
						path.poses[i].pose.position.x=x2
						path.poses[i].pose.position.y=y1
				else:
					if k1>k2:
						path.poses[i].pose.position.x=x2
						path.poses[i].pose.position.y=y1
					else:
						path.poses[i].pose.position.x=x1
						path.poses[i].pose.position.y=y2		
			else:
				if x1<x2:
					if k1>k2:
						path.poses[i].pose.position.x=x1
						path.poses[i].pose.position.y=y2
					else:
						path.poses[i].pose.position.x=x2
						path.poses[i].pose.position.y=y1
				else:
					if k1>k2:
						path.poses[i].pose.position.x=x2
						path.poses[i].pose.position.y=y1
					else:
						path.poses[i].pose.position.x=x1
						path.poses[i].pose.position.y=y2
			poses.append(path.poses[i])
			
			lastGD=GD
			lastj=i
			
		if(i-lastj>50):
			lastj=i
			poses.append(path.poses[i])
		
		i+=10
	poses.append(path.poses[len(path.poses)-1])
	mPath=Path()
	mPath.header.frame_id=path.header.frame_id
	mPath.poses=poses[:]
	return mPath

#再次滤波　主要是对ｐａｔｈ中的拐角进行再次融合变成直角　将小距离的目标点变成大距离的目标点
def Lvbo(path,D):
	
	mPath=Path()
	mPath.header.frame_id=path.header.frame_id
	
	poses=path.poses
	P=len(poses)
	newPoses=[]
	
	newPoses.append(poses[0])
	print '---------------------------'
	#如果只有或者小于三个点
	if P<4:
		for i in range(1,P):
			newPoses.append(poses[i])
		mPath.poses=newPoses[:]
		return mPath
	#从第三个点开始计算
	i=2
	FLAG=0
	while i<(P-1):
		d=canculateDistance(poses[i],poses[i-1])
		if (d<D) :
			if d<0.25 and ((abs(poses[i-1].pose.position.x-poses[i].pose.position.x)<0.05) or (abs(poses[i-1].pose.position.y-poses[i].pose.position.y)<0.05)):
					newPoses.append(poses[i-1])
					i+=2
					FLAG=3
					
			else:
				#直线角度若相差过小　可能产生尖端
				k1=canculate_G_C_Angle(poses[i-2],poses[i-1])
				k2=canculate_G_C_Angle(poses[i],poses[i+1])
				#30度以内
				if abs(normalize_angle(k1-k2))<0.5:
					newPoses.append(poses[i-1])
					i+=2
					FLAG=2
				else:
					result=CrossPoint(poses[i-2],poses[i-1],poses[i],poses[i+1])
					poses[i-1].pose.position.x=result[0]
					poses[i-1].pose.position.y=result[1]
					newPoses.append(poses[i-1])
					i+=2
					FLAG=1
		else:
			newPoses.append(poses[i-1])
			i+=1
			FLAG=2
	if i==(P-1):
		newPoses.append(poses[P-2])
		newPoses.append(poses[P-1])
		mPath.poses=newPoses[:]
		return mPath
	if i==P :
		newPoses.append(poses[P-1])
		mPath.poses=newPoses[:]
		return mPath
#最后的目标点选择　将小距离的点　变成长距离的目标点
def ChooseMainPath(path):
	mPath=Path()
	mPath.header.frame_id=path.header.frame_id
	
	poses=path.poses
	P=len(poses)
	if P<3:
		return path
	newPoses=[]
	newPoses.append(poses[0])
	lastAngle=canculate_G_C_Angle(poses[1],poses[0])
	for i in range(1,P-1):
		angle=canculate_G_C_Angle(poses[i+1],poses[i])
		#print 'agnle-'
		errA=abs(angle-lastAngle)
		if errA>6:
			errA=6.283-errA
		#print errA
		if errA>0.1:
			newPoses.append(poses[i])
			lastAngle=angle
	newPoses.append(poses[P-1])
	mPath.poses=newPoses[:]
	return mPath
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res
