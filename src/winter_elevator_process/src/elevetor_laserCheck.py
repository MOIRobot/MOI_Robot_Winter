#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi
import time
import commands
import dynamic_reconfigure.client
import os
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from math import radians, copysign, sqrt, pow, pi,cos
import tf
import math
import PyKDL

from std_msgs.msg import String

global doorState


rospy.init_node('laser_check', anonymous=False)


def LaserScanCallback(lasermsg):
	global doorState
	#num=len(lasermsg.ranges)
	#print lasermsg.ranges[3590]
	#print lasermsg.ranges[900],lasermsg.ranges[1800]
	min_num=3600-450
	max_num=3600+450
	i=min_num
	count=0
	while i<max_num:
		#print lasermsg.ranges[i]
		if i>=3600:
			if (lasermsg.ranges[i-3600])>2.3:
				count=count+1
		else:
			if (lasermsg.ranges[i])>2.3:
				count=count+1
		i=i+5
	#print count	
	if count<15:
		doorState=False
	else:
		doorState=True
def callback(config):
	rospy.loginfo("request {enabled},{cost_scaling_factor},{inflation_radius}".format(**config))
#将四元数转化为角度信息	
def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
def get_odom():
	 while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			return Point(*trans),quat_to_angle(Quaternion(*rot))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res
#计算两个点中间的距离
def canculateDistance(pose1,pose2):
	return sqrt(pow((pose1.pose.position.x-pose2.pose.position.x), 2)+pow(pose1.pose.position.y-pose2.pose.position.y,2))
#频率
RATE =rospy.get_param("~RATE",20)
#旋转时的角度计算
#最大角速度
MAX_ANGULAR_Z=rospy.get_param("~MAX_ANGULAR_Z",0.6)
#最小角速度
MIN_ANGULAR_Z=rospy.get_param("~MIN_ANGULAR_Z",0.1)
#加速度
ACC_ANGULAR_Z=rospy.get_param("~ACC_ANGULAR_Z",0.4)
#直接加速然后减速后的角度值
MODE1_ANGLE=MAX_ANGULAR_Z*MAX_ANGULAR_Z/ACC_ANGULAR_Z
#直接加速或者减速产生的角度
MODE2_ANGLE=MODE1_ANGLE/2.0
#角度误差多少范围内接受
ANGULAR_Z_ERR=rospy.get_param("~ANGULAR_Z_ERR",0.1)


#旋转时的角度计算
#最大线速度
MAX_LINEAR_X=rospy.get_param("~MAX_LINEAR_X",0.4)
#最小线速度
MIN_LINEAR_X=rospy.get_param("~MIN_LINEAR_X",0.05)
#最大线速度加速度
ACC_LINEAR_X=rospy.get_param("~ACC_LINEAR_X",0.4)
#直接加速然后减速后的距离
MODE1_DIS=MAX_LINEAR_X*MAX_LINEAR_X/ACC_LINEAR_X
#直接加速或者减速产生的距离
MODE2_DIS=MODE1_DIS/2.0
#距离误差多少范围内接受
LINEAR_X_ERR=rospy.get_param("~LINEAR_X_ERR",0.05)
r = rospy.Rate(RATE)

def rotateToGoal(goalAngle):
	
	global POSES
	#获取当前位置
	(position, rotation) = get_odom()
	#获取当前的角度	
	current_angle = float(rotation)
	#判断左转还是右转
	turn_angle=normalize_angle(goalAngle-current_angle)
	
	ROTATE_ACC_SPEED=0.4
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
			if(abs(turn_angle)>(abs(O_turn_angle)/2.0)) :
				move_cmd.angular.z+=ROTATE_ACC_SPEED/RATE
			else:
				if abs(move_cmd.angular.z)>MIN_ANGULAR_Z:
					move_cmd.angular.z-=ROTATE_ACC_SPEED/RATE
			
	else:
		#模式２ 直接加速　匀速　然后减速
		while abs(turn_angle)>ANGULAR_Z_ERR   and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
			if(abs(turn_angle)>MODE2_ANGLE):
				if abs(move_cmd.angular.z)>=MAX_ANGULAR_Z:
					move_cmd.angular.z+=0.0
				else:
					move_cmd.angular.z+=ROTATE_ACC_SPEED/RATE
			else:
				if abs(move_cmd.angular.z)>MIN_ANGULAR_Z:
					move_cmd.angular.z-=ROTATE_ACC_SPEED/RATE 
		
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)


rospy.Subscriber('/scan',LaserScan,LaserScanCallback)
vpub=rospy.Publisher('/robot_state',String,queue_size=1)
mapchange=rospy.Publisher('/change_map',UInt8MultiArray,queue_size=1)
listener = tf.TransformListener()
r = rospy.Rate(20)

'''
mapchangmsg=UInt8MultiArray()
while not rospy.is_shutdown():
	data=[1,0]
	mapchangmsg.data=data
	mapchange.publish(mapchangmsg)
	time.sleep(1)
exit(0)
'''

move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")
move_base.wait_for_server(rospy.Duration(180))
rospy.loginfo("Connected to move base server start testing")
client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer", timeout=30, config_callback=callback)
cmd_vel = rospy.Publisher('/smooth_cmd_vel', Twist, queue_size=5)


def radiuschangeToNormal():
	client.update_configuration({"enabled":True,"cost_scaling_factor":10,"inflation_radius":0.8})
def radiuschangeToElevator():
	client.update_configuration({"enabled":True,"cost_scaling_factor":10,"inflation_radius":0.2})

def move(goal):
	move_base.send_goal(goal)
	while not  rospy.is_shutdown():
		(position, rotation) = get_odom()
		dis=sqrt(pow((position.x-goal.target_pose.pose.position.x), 2)+pow(position.y-goal.target_pose.pose.position.y,2))
		print dis
		if dis<=0.15:
			move_base.cancel_goal()
			#让机器人停止运动
			cmd_vel.publish(Twist())
			#跳出循环
			break
		time.sleep(0.2)
		#转到电梯朝向	
	fgoal=quat_to_angle(goal.target_pose.pose.orientation)
	rotateToGoal(fgoal)
	#move_base.wait_for_result(rospy.Duration(20*60)) 
	#state = move_base.get_state()
	#if state == GoalStatus.SUCCEEDED:
	#	rospy.loginfo("Goal succeeded!")	
def WaitDoorOpen():
	global doorState
	try:
		while doorState==False and not rospy.is_shutdown():
			time.sleep(0.5)
			print 'waiting for elevator to open'
	except:
		return 
	
	
def check():
	global doorState
	
	waitPose=[5.43,-1.5, -0.654,0.757,
						5.43,0.65,  0.71, 0.704,
						2.646,0.7, 0.71, 0.704,
						2.646,-1.45, -0.654, 0.757]
	goalFrontE=[]
	
	goalInEPose=[5.43,-3.15, 0.71, 0.704,
						5.47,2.35,  -0.708, 0.706,
						2.646,2.35, -0.708, 0.706,
						2.646,-3.15, 0.71, 0.704]
	goalInE=[]
	for i in range(4):
		goal=MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x=waitPose[i*4]
		goal.target_pose.pose.position.y=waitPose[i*4+1]
		goal.target_pose.pose.position.z=0.0
		goal.target_pose.pose.orientation.x=0.0
		goal.target_pose.pose.orientation.y=0.0
		goal.target_pose.pose.orientation.z=waitPose[i*4+2]
		goal.target_pose.pose.orientation.w=waitPose[i*4+3]
		goalFrontE.append(goal)
	for i in range(4):
		goal=MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x=goalInEPose[i*4]
		goal.target_pose.pose.position.y=goalInEPose[i*4+1]
		goal.target_pose.pose.position.z=0.0
		goal.target_pose.pose.orientation.x=0.0
		goal.target_pose.pose.orientation.y=0.0
		goal.target_pose.pose.orientation.z=goalInEPose[i*4+2]
		goal.target_pose.pose.orientation.w=goalInEPose[i*4+3]
		goalInE.append(goal)

	goal2out = MoveBaseGoal()
	goal2out.target_pose.header.frame_id = 'map'
	goal2out.target_pose.header.stamp = rospy.Time.now()
	goal2out.target_pose.pose.position.x=5.43
	goal2out.target_pose.pose.position.y=0.0
	goal2out.target_pose.pose.position.z=0.0
	#angle 0.604
	goal2out.target_pose.pose.orientation.x=0.0
	goal2out.target_pose.pose.orientation.y=0.0
	goal2out.target_pose.pose.orientation.z=-0.654
	goal2out.target_pose.pose.orientation.w=0.757

	while not rospy.is_shutdown():
		sel=input("choose goal:")
		move(goalFrontE[sel])
		radiuschangeToElevator()
		os.system("play waitOpen.mp3")
		WaitDoorOpen()
		os.system("play goingInElevator.mp3")
		move(goalInE[sel])
		time.sleep(5)
		os.system("play waitOpen.mp3")
		WaitDoorOpen()
		os.system("play goingOutElevator.mp3")
		move(goal2out)
		radiuschangeToNormal()
			
if __name__ == '__main__':
	check()
	
