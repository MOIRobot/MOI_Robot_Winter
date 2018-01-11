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
def callback(config):
	rospy.loginfo("request {enabled},{cost_scaling_factor},{inflation_radius}".format(**config))

rospy.init_node('laser_check', anonymous=False)
rospy.Subscriber('/scan',LaserScan,LaserScanCallback)
vpub=rospy.Publisher('/robot_state',String,queue_size=1)
mapchange=rospy.Publisher('/change_map',UInt8MultiArray,queue_size=1)

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


def radiuschangeToNormal():
	client.update_configuration({"enabled":True,"cost_scaling_factor":10,"inflation_radius":0.8})
def radiuschangeToElevator():
	client.update_configuration({"enabled":True,"cost_scaling_factor":10,"inflation_radius":0.2})

def move(goal):
	move_base.send_goal(goal)
	move_base.wait_for_result(rospy.Duration(20*60)) 
	state = move_base.get_state()
	if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("Goal succeeded!")	
def WaitDoorOpen():
	global doorState
	try:
		while doorState==False:
			time.sleep(0.5)
			print 'waiting for elevator to open'
	except:
		return 
	
	
def check():
	global doorState
	
	waitPose=[5.43,-1.45, -0.654,0.757,
						5.43,0.65,  0.71, 0.704,
						2.646,0.7, 0.71, 0.704,
						2.646,-1.45, -0.654, 0.757]
	goalFrontE=[]
	
	goalInEPose=[5.43,-3.0, 0.71, 0.704,
						5.47,2.25,  -0.708, 0.706,
						2.646,2.25, -0.708, 0.706,
						2.646,-3.0, 0.71, 0.704]
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
		time.sleep(15)
		os.system("play waitOpen.mp3")
		WaitDoorOpen()
		os.system("play goingOutElevator.mp3")
		move(goal2out)
		radiuschangeToNormal()
			
if __name__ == '__main__':
	check()
	
