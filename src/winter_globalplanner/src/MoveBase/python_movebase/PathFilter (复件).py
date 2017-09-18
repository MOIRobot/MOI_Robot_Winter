#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from math import radians, copysign, sqrt, pow, pi,cos
from nav_msgs.msg import Path
import PyKDL
import tf
import math

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

