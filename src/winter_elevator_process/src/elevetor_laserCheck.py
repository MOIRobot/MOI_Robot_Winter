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
	if count<20:
		doorState=False
	else:
		doorState=True

rospy.init_node('laser_check', anonymous=False)
rospy.Subscriber('/scan',LaserScan,LaserScanCallback)
vpub=rospy.Publisher('/robot_state',String,queue_size=1)
r = rospy.Rate(20)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rospy.loginfo("Waiting for move_base action server...")
move_base.wait_for_server(rospy.Duration(180))
rospy.loginfo("Connected to move base server start testing")
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
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x=5.53
	goal.target_pose.pose.position.y=-1.45
	goal.target_pose.pose.position.z=0.0
	#angle 0.604
	goal.target_pose.pose.orientation.x=0.0
	goal.target_pose.pose.orientation.y=0.0
	goal.target_pose.pose.orientation.z=-0.654
	goal.target_pose.pose.orientation.w=0.757
	
	goal2 = MoveBaseGoal()
	goal2.target_pose.header.frame_id = 'map'
	goal2.target_pose.header.stamp = rospy.Time.now()
	goal2.target_pose.pose.position.x=5.53
	goal2.target_pose.pose.position.y=0.65
	goal2.target_pose.pose.position.z=0.0
	#angle 0.604
	goal2.target_pose.pose.orientation.x=0.0
	goal2.target_pose.pose.orientation.y=0.0
	goal2.target_pose.pose.orientation.z=0.71
	goal2.target_pose.pose.orientation.w=0.704
	
	
	goal3 = MoveBaseGoal()
	goal3.target_pose.header.frame_id = 'map'
	goal3.target_pose.header.stamp = rospy.Time.now()
	goal3.target_pose.pose.position.x=2.646
	goal3.target_pose.pose.position.y=0.7
	goal3.target_pose.pose.position.z=0.0
	#angle 0.604
	goal3.target_pose.pose.orientation.x=0.0
	goal3.target_pose.pose.orientation.y=0.0
	goal3.target_pose.pose.orientation.z=0.71
	goal3.target_pose.pose.orientation.w=0.704
	
	goal_in2 = MoveBaseGoal()
	goal_in2.target_pose.header.frame_id = 'map'
	goal_in2.target_pose.header.stamp = rospy.Time.now()
	goal_in2.target_pose.pose.position.x=5.6
	goal_in2.target_pose.pose.position.y=2.4
	goal_in2.target_pose.pose.position.z=0.0
	#angle 0.604
	goal_in2.target_pose.pose.orientation.x=0.0
	goal_in2.target_pose.pose.orientation.y=0.0
	goal_in2.target_pose.pose.orientation.z=-0.708
	goal_in2.target_pose.pose.orientation.w=0.706

	while True:
		sel=input("choose goal:")
		if sel==1:
			print 'going to goal 1'
			move(goal)
		elif sel==2:
			print 'going to goal2'
			move(goal2)
		elif sel==3:
			print 'going to goal3'
			move(goal3)
		elif sel=='q':
			return 
	'''
	move(goal_in2)
	time.sleep(15)
	while doorState==False:
		time.sleep(0.5)
		print 'waiting for elevator to open'
	print 'going to the elevator'
	move(goal)
	'''
	'''
	goal2 = MoveBaseGoal()
	goal2.target_pose.header.frame_id = 'map'
	goal2.target_pose.header.stamp = rospy.Time.now()
	goal2.target_pose.pose.position.x=5.53
	goal2.target_pose.pose.position.y=0.65
	goal2.target_pose.pose.position.z=0.0
	#angle 0.604
	goal2.target_pose.pose.orientation.x=0.0
	goal2.target_pose.pose.orientation.y=0.0
	goal2.target_pose.pose.orientation.z=0.71
	goal2.target_pose.pose.orientation.w=0.704
	move(goal2)
	while not rospy.is_shutdown():
		r.sleep()	
	'''
if __name__ == '__main__':
	check()
	
