#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from math import radians, copysign, sqrt, pow, pi,cos
from nav_msgs.msg import Path
import PyKDL
import tf
import math
import PathFilter

#平滑速度 返回目标点的位置 与方向
#0.1745 10度
def averageSpeed(path):
	Length=len(path.poses)
	finalpath=[]
	last=PathFilter.quat_to_angle(path.poses[0].pose.orientation)
	goalposition=[]
	for i in range(1,Length):
		GD=PathFilter.quat_to_angle(path.poses[i].pose.orientation)
		#if()
		last=GD

NewPath=False
POSES=[]
def PathCallback(path):
	
		newpath=PathFilter.newPathFromAStar(path)
		mPath2.publish(newpath)
		#两次滤波对路径进行规整
		newpath=PathFilter.Lvbo(newpath,0.5)
		newpath=PathFilter.Lvbo(newpath,0.8)
		mPath.publish(newpath)	
		poses=newpath.poses
		
		POSES=poses[:]
		NewPath=True
		
rospy.init_node('pathlistener', anonymous=False)
rospy.Subscriber('/planner/planner/plan',Path,PathCallback)
mPath=rospy.Publisher('/mplannerplan',Path,queue_size=5)

mPath2=rospy.Publisher('/mplannerplan1',Path,queue_size=5)

tf_listener = tf.TransformListener()
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

r = rospy.Rate(rate)
ROTATE_SPEED=1.2
X_SPEED=0.4



#频率
RTAE = 100
#旋转时的角度计算
#最大角速度
MAX_ANGULAR_Z=0.8
#加速度
ACC_ANGULAR_Z=0.8
#直接加速然后减速后的角度值
MODE1_ANGLE=MAX_ANGULAR_Z*MAX_ANGULAR_Z/ACC_ANGULAR_Z
#直接加速或者减速产生的角度
MODE2_ANGLE=MODE1_ANGLE/2.0



#传入目标位置　目标点方向四元素　将机器人转动到向该目标点运动的方向
def rotateToGoalDirection(pose,rot,final):
	goalAngle=0.0
	#获取当前位置
	(position, rotation) = get_odom()
	if(final!=True):
		#获取机器人目标的位置　角度
		GX=pose.position.x
		GY=pose.position.y
		
		CX=position.x
		CY=position.y
		#计算坐标差值
	
		#计算两个目标点与当前点的角度差值
		goalAngle=PathFilter.canculateAngle(GX,GY,CX,CY)
	else:
		goalAngle=PathFilter.quat_to_angle(rot)
	goalAngleC=0.0
	last_angleC=0.0
	#将四元素转化为角度
	#goalAngle=quat_to_angle(rot)
	
	if(goalAngle<0.0):
		goalAngleC=2*pi+goalAngle
	else:
		goalAngleC=goalAngle
	#rospy.loginfo("Goal Angle is %f",goalAngleC)
	# Track the last angle measured
	last_angle = float(rotation)
	# Track how far we have turned
	if(rotation<0.0):
		last_angleC=2*pi+rotation
	else:
		last_angleC=last_angle
	#rospy.loginfo("curretn Angle is %f",rotation)
	turn_angle=goalAngleC-last_angleC
	
	move_cmd=Twist()
	if (0<=turn_angle and turn_angle<=pi) or ((-2*pi)<=turn_angle and turn_angle<=(0-pi)):
		move_cmd.angular.z = ROTATE_SPEED
	else:
		move_cmd.angular.z = 0-ROTATE_SPEED
	#转到运动方向上
	turn_angle=goalAngle-last_angle
	#0.02 大约１．１４度　
	while abs(turn_angle)>0.05 and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)

#传入目标位置　目标点
def rotateToMoveDirection(pose):
	goalAngle=0.0
	#获取当前位置
	(position, rotation) = get_odom()
	
	#计算目标点与当前点的全局坐标系下的角度值
	goalAngle=PathFilter.canculateAngle(pose.position.x,pose.position.y,position.x,position.y)
	
	current_angle = float(rotation)
	
	#判断左转还是右转
	turn_angle=normalize_angle(goalAngle-current_angle)
	
	
	ROTATE_ACC_SPEED=ACC_ANGULAR_Z
	if turn_angle<0.0:
		ROTATE_ACC_SPEED=0-ROTATE_ACC_SPEED

	#记录最初的旋转角
	O_turn_angle=turn_angle
	
	move_cmd=Twist()
	#转到运动方向上
	if abs(O_turn_angle)<MODE1_ANGLE:
		#模式１ 直接加速　然后减速
		while abs(turn_angle)>0.02   and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
			if(abs(turn_angle)>(abs(O_turn_angle)/2.0)):
				move_cmd.angular.z+=ROTATE_ACC_SPEED/RATE
			else:
				move_cmd.angular.z-=ROTATE_ACC_SPEED/RATE
			
	else:
		#模式２ 直接加速　　匀速　然后减速
		while abs(turn_angle)>0.02   and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
			if(abs(turn_angle)>MODE1_ANGLE):
				if abs(move_cmd.angular.z)>=MAX_ANGULAR_Z:
					ROTATE_ACC_SPEED/RATE=0.0
				move_cmd.angular.z+=ROTATE_ACC_SPEED/RATE
				
					
			else:
				move_cmd.angular.z-=ROTATE_ACC_SPEED/RATE
		
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)	
	

#传入目标位置　目标点方向四元素　将机器人转动到向该目标点运动的方向
def rotateToGoalDirection1(goalAngle,rotation):
	
	if(abs(goalAngle-rotation)<0.06):
		return 0.0
	if(goalAngle<0.0):
		goalAngleC=2*pi+goalAngle
	else:
		goalAngleC=goalAngle
	#rospy.loginfo("Goal Angle is %f",goalAngleC)
	# Track the last angle measured
	last_angle = float(rotation)
	# Track how far we have turned
	if(rotation<0.0):
		last_angleC=2*pi+rotation
	else:
		last_angleC=last_angle
		
	turn_angle=goalAngleC-last_angleC
	if (0<=turn_angle and turn_angle<=pi) or ((-2*pi)<=turn_angle and turn_angle<=(0-pi)):
		return ROTATE_SPEED
	else:
		return (0-ROTATE_SPEED)
	
#向目标位置点运动　是在运动方向基础上　前进
#传入参数　目标点的位置 只从机器人前进方向运动
def	moveToGoalX(pose):
	#获取机器人目标的位置　角度
	GX=pose.position.x
	GY=pose.position.y
	(position, rotation) = get_odom()
	x_start=position.x
	y_start=position.y
	move_cmd=Twist()
	move_cmd.linear.x=1
	distance = 0
	#到目标点的位置
	goal_distance=sqrt(pow((GX - x_start), 2)+pow((GY - y_start), 2))
	while distance<goal_distance and not rospy.is_shutdown():
		cmd_vel.publish(move_cmd)
		r.sleep()
		(position, rotation) = get_odom()
		distance=sqrt(pow((position.x - x_start), 2)+pow((position.y - y_start), 2))
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)
	
#向目标位置点运动　是在运动方向基础上　前进
#传入参数　目标点的位置 只从机器人前进方向运动
#加上旋转的方式 位置
def	moveToGoalRotate(pose,pose2):
	
	#获取机器人目标的位置　角度
	GX=pose.position.x
	GY=pose.position.y
	(position, rotation) = get_odom()
	x_start=position.x
	y_start=position.y
	move_cmd=Twist()
	move_cmd.linear.x=0.4
	distance = 0
	
	#目标点与下一目标点的方向
	NextGoalAngle=canculate_G_C_Angle(pose2,pose)
	#到目标点的位置
	goal_distance=sqrt(pow((GX - x_start), 2)+pow((GY - y_start), 2))
	
	while distance<goal_distance and not rospy.is_shutdown():
		(position, rotation) = get_odom()
		distance=sqrt(pow((position.x - x_start), 2)+pow((position.y - y_start), 2))
		if ((goal_distance-distance)<0.5):	
				move_cmd.angular.z=rotateToGoalDirection1(NextGoalAngle,rotation)	
		cmd_vel.publish(move_cmd)
		r.sleep()
#传入参数　目标点的位置 时间　全向运动到机器人前进方向运动	
def moveToGoalXY(pose):
	#获取机器人目标的位置　角度
	GX=pose.position.x
	GY=pose.position.y
	(position, rotation) = get_odom()
	x_start=position.x
	y_start=position.y
	move_cmd=Twist()
	
	#获取
	#vx=move_cmd.linear.x*math.cos(rotation)-move_cmd.linear.y*math.sin(rotation)
	#vy=move_cmd.linear.x*math.sin(rotation)+move_cmd.linear.y*math.cos(rotation)
	
	#获取该方向的速度比例
	if abs(GX-x_start)>=abs(GY-y_start):
		if GX-x_start>0:
			vx=0.5
			vy=(GY-y_start)/(GX-x_start)*vx
		else:
			vx=-0.5
			vy=(GY-y_start)/(GX-x_start)*vx
	else:
		if (GY-y_start)>0:
			vy=0.5
			vx=(GX-x_start)/(GY-y_start)*vy
		else:
			vy=-0.5
			vx=(GX-x_start)/(GY-y_start)*vy
	
	
	#求出机器人的运动的速度
	move_cmd.linear.x=vx*math.cos(rotation)+vy*math.sin(rotation)
	move_cmd.linear.y=vy*math.cos(rotation)-vx*math.sin(rotation)
	
	distance = 0
	#到目标点的位置
	goal_distance=sqrt(pow((GX - x_start), 2)+pow((GY - y_start), 2))
	while distance<goal_distance and not rospy.is_shutdown():
		cmd_vel.publish(move_cmd)
		r.sleep()
		(position, rotation) = get_odom()
		distance=sqrt(pow((position.x - x_start), 2)+pow((position.y - y_start), 2))
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)
	


#求两个角度之间的差值	
def normalize_angle(angle):
    res = angle
    if res > pi:
        res -= 2.0 * pi
    elif res < -pi:
        res += 2.0 * pi
    return res
    
#将角度值变成正的
def reflect_angleTo2pi(angle):
	if(angle<0.0):
		return 2*pi+angle
	else:
		return angle
def get_odom():
	(trans, rot) = tf_listener.lookupTransform('/odom','/base_link', rospy.Time(0))
	return Point(*trans),PathFilter.quat_to_angle(Quaternion(*rot))
def movebase(poses):
	global NewPath
	i=1
	rospy.loginfo("has %d goals",len(poses))
	rotateToGoalDirection(poses[0].pose,poses[0].pose.orientation,False)
	while i<(len(poses)) and newPath==False:
		'''
		GX=poses[i].pose.position.x
		GY=poses[i].pose.position.y
		GD=PathFilter.quat_to_angle(poses[i].pose.orientation)
		rospy.loginfo("the %d goal is x:%f y:%f ",i+1,GX,GY)
		d=PathFilter.canculateDistance(poses[i],poses[i-1])
		rospy.loginfo("distance %f ",d)
		#moveToGoalRotate(poses[i].pose,poses[i+1].pose)
		'''
		rotateToGoalDirection(poses[i].pose,poses[i].pose.orientation,False)
		moveToGoalX(poses[i].pose)
		i+=1
	rotateToGoalDirection(poses[i-1].pose,poses[i-1].pose.orientation,True)
if __name__ == '__main__':
	try:
		while True:
			
	except Exception:
		print 'down'
