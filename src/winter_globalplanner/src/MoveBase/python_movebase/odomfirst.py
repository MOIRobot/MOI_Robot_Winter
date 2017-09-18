#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from math import radians, copysign, sqrt, pow, pi,cos
import PyKDL
import tf
import math

def GoalCallback(goal):
	rospy.loginfo("i get a goal x:%f y:%f", goal.pose.position.x,goal.pose.position.y)
	#rotateToGoalDirection(goal.pose,goal.pose.orientation,False)
	#moveToGoalX(goal.pose)
	moveToGoalXY(goal.pose)
	rotateToGoalDirection(goal.pose,goal.pose.orientation,True)

rospy.init_node('pathlistener', anonymous=False)

tf_listener = tf.TransformListener()
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
rate = 20
r = rospy.Rate(rate)
ROTATE_SPEED=0.8
X_SPEED=0.4
rospy.Subscriber('/move_base_simple/goal',PoseStamped,GoalCallback)


def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]

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
		goalAngle=canculateAngle(GX,GY,CX,CY)
	else:
		goalAngle=quat_to_angle(rot)
	goalAngleC=0.0
	last_angleC=0.0
	#将四元素转化为角度
	#goalAngle=quat_to_angle(rot)
	
	if(goalAngle<0.0):
		goalAngleC=2*pi+goalAngle
	else:
		goalAngleC=goalAngle
	rospy.loginfo("Goal Angle is %f",goalAngleC)
	# Track the last angle measured
	last_angle = float(rotation)
	# Track how far we have turned
	if(rotation<0.0):
		last_angleC=2*pi+rotation
	else:
		last_angleC=last_angle
	rospy.loginfo("curretn Angle is %f",rotation)
	turn_angle=goalAngleC-last_angleC
	
	move_cmd=Twist()
	if (0<=turn_angle and turn_angle<=pi) or ((-2*pi)<=turn_angle and turn_angle<=(0-pi)):
		move_cmd.angular.z = ROTATE_SPEED
	else:
		move_cmd.angular.z = 0-ROTATE_SPEED
	#转到运动方向上
	turn_angle=goalAngle-last_angle
	while abs(turn_angle)>0.02 and not rospy.is_shutdown():
			cmd_vel.publish(move_cmd)
			r.sleep()
			(position, rotation) = get_odom()
			turn_angle=goalAngle-rotation
	move_cmd=Twist()
	cmd_vel.publish(move_cmd)
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
	move_cmd.linear.x=0.4
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
	
	
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

def get_odom():
	(trans, rot) = tf_listener.lookupTransform('/odom','/base_link', rospy.Time(0))
	return Point(*trans),quat_to_angle(Quaternion(*rot))
if __name__ == '__main__':
	rospy.spin()
	
