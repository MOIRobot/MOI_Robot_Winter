#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi,cos
from nav_msgs.msg import Path
import PyKDL
import tf
import math
import PathFilter

global POSES
global NewPath
global isForwardObstacle
NewPath=False
isForwardObstacle=False
POSES=[]

def PathCallback(path):
		global POSES
		global NewPath
		newpath=PathFilter.newPathFromAStar(path)
		
		#两次滤波对路径进行规整
		newpath=PathFilter.Lvbo(newpath,0.5)
		newpath=PathFilter.Lvbo(newpath,0.8)
		#mPath2.publish(newpath)
		newpath=PathFilter.Lvbo(newpath,1.0)
		#newpath=PathFilter.Lvbo(newpath,1.6)
		#最后的滤波　选择长距离点
		newpath=PathFilter.ChooseMainPath(newpath)
		mPath.publish(newpath)	
		
		poses=newpath.poses
		
		POSES=poses[:]
		NewPath=True
		
def LaserScanCallback(lasermsg):
	global isForwardObstacle
	num=len(lasermsg.ranges)
	maxangle=110
	obstacleAngle=45
	
	obstacleRange=obstacleAngle*num/maxangle
	i=num/2-obstacleRange
	
	isForwardObstacle=False
	
	while i<(num/2+obstacleRange):
		if(lasermsg.ranges[i]<0.35):
			isForwardObstacle=True
			return 
		i=i+10
	
	
	
	
rospy.init_node('pathlistener', anonymous=False)
rospy.Subscriber('/planner/planner/plan',Path,PathCallback)
rospy.Subscriber('/scan',LaserScan,LaserScanCallback)
mPath=rospy.Publisher('/mplannerplan',Path,queue_size=5)

mPath2=rospy.Publisher('/mplannerplan1',Path,queue_size=5)

#频率
RATE =20.0
#旋转时的角度计算
#最大角速度
MAX_ANGULAR_Z=0.8
#最小角速度
MIN_ANGULAR_Z=0.25
#加速度
ACC_ANGULAR_Z=0.8
#直接加速然后减速后的角度值
MODE1_ANGLE=MAX_ANGULAR_Z*MAX_ANGULAR_Z/ACC_ANGULAR_Z
#直接加速或者减速产生的角度
MODE2_ANGLE=MODE1_ANGLE/2.0
#角度误差多少范围内接受
ANGULAR_Z_ERR=0.02


#旋转时的角度计算
#最大线速度
MAX_LINEAR_X=0.4
#最小线速度
MIN_LINEAR_X=0.05
#最大线速度加速度
ACC_LINEAR_X=0.4
#直接加速然后减速后的距离
MODE1_DIS=MAX_LINEAR_X*MAX_LINEAR_X/ACC_LINEAR_X
#直接加速或者减速产生的距离
MODE2_DIS=MODE1_DIS/2.0
#距离误差多少范围内接受
LINEAR_X_ERR=0.05



tf_listener = tf.TransformListener()
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
r = rospy.Rate(RATE)
ROTATE_SPEED=1.2
X_SPEED=0.4


#传入目标位置　将机器人转动到向该目标点运动的方向
def rotateToMoveDirection(pose):
	#计算目标点与当前点的全局坐标系下的角度值
	(position, rotation) = get_odom()
	goalAngle=PathFilter.canculateAngle(pose.pose.position.x,pose.pose.position.y,position.x,position.y)
	rotateToGoal(goalAngle)
#转动到目标点的方向
def rotateToGoalDirection(pose):
	goalAngle=PathFilter.quat_to_angle(pose.pose.orientation)
	rotateToGoal(goalAngle)
#转动到指定的方向
def rotateToGoal(goalAngle):
	
	global POSES
	global NewPath
	
	#如果出现新的路径　直接返回退出
	if(NewPath):
		return
	
	#获取当前位置
	(position, rotation) = get_odom()
	#获取当前的角度	
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
		while abs(turn_angle)>ANGULAR_Z_ERR  and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
			if(abs(turn_angle)>(abs(O_turn_angle)/2.0)) and not NewPath:
				#如果没有出现新的路径　则正常执行
				move_cmd.angular.z+=ROTATE_ACC_SPEED/RATE
			else:
				if abs(move_cmd.angular.z)>MIN_ANGULAR_Z:
					move_cmd.angular.z-=ROTATE_ACC_SPEED/RATE
				else:
					#如果是出现新的路径则直接速度归０　跳出循环
					if(NewPath):
						move_cmd=Twist()
						cmd_vel.publish(move_cmd)
						return 
			
	else:
		#模式２ 直接加速　匀速　然后减速
		while abs(turn_angle)>ANGULAR_Z_ERR   and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
			if(abs(turn_angle)>MODE2_ANGLE) and not NewPath:
				if abs(move_cmd.angular.z)>=MAX_ANGULAR_Z:
					move_cmd.angular.z+=0.0
				else:
					move_cmd.angular.z+=ROTATE_ACC_SPEED/RATE
			else:
				if abs(move_cmd.angular.z)>MIN_ANGULAR_Z:
					move_cmd.angular.z-=ROTATE_ACC_SPEED/RATE
				else:
					#如果是出现新的路径则直接速度归０　跳出循环
					if(NewPath):
						move_cmd=Twist()
						cmd_vel.publish(move_cmd)
						return 
		
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)
	#print 'rotate end'

#向目标位置点运动　是在运动方向基础上　前进
#传入参数　目标点的位置 只从机器人前进方向运动
def	moveToGoal_Forward(pose):
	
	global POSES
	global NewPath
	global isForwardObstacle
	
	#如果出现新的路径　直接返回退出
	if(NewPath):
		return
	
	#获取机器人目标的位置　角度
	GX=pose.pose.position.x
	GY=pose.pose.position.y
	(position, rotation) = get_odom()
	x_start=position.x
	y_start=position.y
	move_cmd=Twist()
	
	#到目标点的位置
	goal_distance=sqrt(pow((GX - x_start), 2)+pow((GY - y_start), 2))
	distance=0.0
	rospy.loginfo("goal_distance:%f",goal_distance)
	if goal_distance<MODE1_DIS:
		while distance<goal_distance and not rospy.is_shutdown():
			#模式１　加速　然后减速
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			distance=sqrt(pow((position.x - x_start), 2)+pow((position.y - y_start), 2))
			if ((distance<(goal_distance/2.0)) and not NewPath and not isForwardObstacle):
				move_cmd.linear.x+=ACC_LINEAR_X/RATE
			else:
				if move_cmd.linear.x<=MIN_LINEAR_X:
					move_cmd.linear.x=MIN_LINEAR_X
					if(NewPath):
						#如果是出现新的路径　则当速度降到０时　退出函数
						move_cmd=Twist()
						cmd_vel.publish(move_cmd)
						return
					if(isForwardObstacle):
						#如果是前方出现障碍物　则当速度降到０时　原地等待
						move_cmd=Twist()
						cmd_vel.publish(move_cmd)
						while isForwardObstacle:
							r.sleep()
				else:
					move_cmd.linear.x-=ACC_LINEAR_X/RATE
	else:
		while distance<goal_distance and not rospy.is_shutdown():
			#模式１　加速　匀速　然后减速
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			distance=sqrt(pow((position.x - x_start), 2)+pow((position.y - y_start), 2))
			if (((goal_distance-distance)>MODE2_DIS)and not NewPath and not isForwardObstacle):
				if move_cmd.linear.x>=MAX_LINEAR_X:
					move_cmd.linear.x=MAX_LINEAR_X
				else:
					move_cmd.linear.x+=ACC_LINEAR_X/RATE
			else:
				if move_cmd.linear.x<=MIN_LINEAR_X:
					move_cmd.linear.x=MIN_LINEAR_X
					if(NewPath):
						#如果是出现新的路径　则当速度降到０时　退出函数
						move_cmd=Twist()
						cmd_vel.publish(move_cmd)
						return
					if(isForwardObstacle):
						#如果是前方出现障碍物　则当速度降到０时　原地等待
						move_cmd=Twist()
						cmd_vel.publish(move_cmd)
						while isForwardObstacle:
							r.sleep()
				else:
					move_cmd.linear.x-=ACC_LINEAR_X/RATE
	
	#rospy.loginfo("distance error  %f ",goal_distance-distance)			
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)
	
#向目标位置点运动　是在运动方向基础上　前进
#传入参数　目标点的位置 只从机器人前进方向运动
#加上旋转的方式 位置
def	moveToGoalRotate(pose,pose2):	
	#传入参数　目标点的位置 时间　全向运动到机器人前进方向运动	
	a=1
def moveToGoalXY(pose):
	#获取机器人目标的位置　角度
	b=2
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

def movebase():
	global POSES
	global NewPath
	while not rospy.is_shutdown():
		#如果没有出现新的路径则等待
		try:
			while not NewPath:
				r.sleep()
		except Exception:
			print 'down'
		NewPath=False
		i=1
		rospy.loginfo("has %d goals",len(POSES)-1)
		while i<len(POSES):
			'''
			GX=poses[i].pose.position.x
			GY=poses[i].pose.position.y
			GD=PathFilter.quat_to_angle(poses[i].pose.orientation)
			rospy.loginfo("the %d goal is x:%f y:%f ",i+1,GX,GY)
			d=PathFilter.canculateDistance(poses[i],poses[i-1])
			rospy.loginfo("distance %f ",d)
			'''
			rotateToMoveDirection(POSES[i])
			moveToGoal_Forward(POSES[i])
			i+=1
		rotateToGoalDirection(POSES[i-1])
			
def get_odom():
	(trans, rot) = tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))
	return Point(*trans),PathFilter.quat_to_angle(Quaternion(*rot))
if __name__ == '__main__':
	movebase()
	
