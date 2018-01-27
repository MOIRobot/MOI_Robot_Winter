#!/usr/bin/env python 
#coding=utf-8


#接收cmdvel信息并发布到carspeed话题上


import roslib; roslib.load_manifest('winter_simulation') 
import rospy
import tf.transformations
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from math import sqrt, atan2
import thread
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


#listener callback
VX=0.0
VY=0.0
VZ=0.0

l_time=-1.0

max_x=0.0
max_acc_x=0.0
max_th=0.0
max_acc_th=0.0
delta_t=1110.0
lx=0.0
lth=0.0

def callback(msg):
	global pub
	global VX
	global VY
	global VZ
	global l_time
	global max_x
	global max_acc_x
	global max_th
	global max_acc_th
	global lx
	global lth
	global delta_t

	VX=msg.twist.twist.linear.x
	VY=msg.twist.twist.linear.y
	VZ=msg.twist.twist.angular.z
def tt():
	global pub
	global VX
	global VY
	global VZ
	global l_time
	global max_x
	global max_acc_x
	global max_th
	global max_acc_th
	global lx
	global lth
	global delta_t
	seconds = rospy.get_time()
	if (l_time >-1.0):
		delta_t=seconds-l_time
	l_time=seconds

	if VX>=max_x:
		max_x=VX
	if VZ>=max_th:
		max_th=VZ
	if(l_time >-1.0):
		accx=(VX-lx)/delta_t
		if abs(accx)>max_acc_x:
			max_acc_x=abs(accx)
		accth=(VZ-lth)/delta_t
		if abs(accth)>max_acc_th:
			max_acc_th=abs(accth)
	lx=VX
	lth=VZ
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


rospy.init_node('robot_driver',anonymous=True)
rospy.Subscriber("/odom",Odometry,callback)
listener = tf.TransformListener()
rate=rospy.Rate(5)
while not rospy.is_shutdown():
	(position, rotation) = get_odom()
	tt()
	print "max_x",max_x,"max_acc_x",max_acc_x,"max_th",max_th,"max_acc_th",max_acc_th
	print "XP ",position.x,"YP",position.y,"angle",rotation
	rate.sleep()		
def nodeShutdown():
	print('the fake robot driver is down!')

