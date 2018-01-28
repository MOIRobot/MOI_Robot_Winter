#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import time
from std_msgs.msg import String
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

global doorState

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

rospy.init_node('laser_check', anonymous=False)
#rospy.Subscriber('/scan',LaserScan,LaserScanCallback)
vpub=rospy.Publisher('/robot_state',String,queue_size=1)
r = rospy.Rate(20)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")
move_base.wait_for_server(rospy.Duration(180))
rospy.loginfo("Connected to move base server start testing")

def callback(config):
	rospy.loginfo("request {enabled},{cost_scaling_factor},{inflation_radius}".format(**config))
client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer", timeout=30, config_callback=callback)

def radiuschangeToNormal():
	client.update_configuration({"enabled":True,"cost_scaling_factor":10,"inflation_radius":0.8})
def radiuschangeToElevator():
	client.update_configuration({"enabled":True,"cost_scaling_factor":10,"inflation_radius":0.2})
if __name__ == '__main__':
	while True:
		a=input("af:")
		if a==1:
			radiuschangeToNormal()
		elif a==2:
			radiuschangeToElevator()
		else:
			exit(0)
			
	
