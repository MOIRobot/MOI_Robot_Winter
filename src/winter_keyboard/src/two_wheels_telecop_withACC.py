#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('winter_keyboard') 
import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import String
import  os  
import  sys
import  tty, termios
import time    

import pyttsx
def onEnd(name, completed):
		print 'end'
class TSpeak:
	def __init__(self):
		self.engine=pyttsx.init()
		self.engine.setProperty('rate', 120)
		self.engine.connect('finished-utterance', onEnd)
	def say(self,words):
		self.engine.say(words)
		#self.engine.runAndWait()
		#self.engine.runAndWait()
		self.engine.startLoop()
		#self.engine.startLoop(False)
		# engine.iterate() must be called inside externalLoop()
		#externalLoop()
		#self.engine.endLoop()
	def wait(self):
		self.engine.runAndWait()
	def onsEnd(self):
		self.engine.endLoop()
cmd = Twist()
<<<<<<< HEAD
pub = rospy.Publisher('smooth_cmd_vel',Twist,queue_size=1)


=======
pub = rospy.Publisher('smooth_cmd_vel',Twist,queue_size=20)
man=TSpeak()
>>>>>>> 53bbca895b8b1e257465748ae84cb74df138cfbc
global CurrentSpeedX
global CurrentSpeedY
global CurrentRotate
global ACC
global RotateAcc
global MAXSPEED
global MAXROTATESPEED

MAXSPEED=0.3
MAXROTATESPEED=0.5

ACC=0.3
RotateAcc=0.3
#控制频率
global ControllerFrequecny
ControllerFrequecny=10

CurrentSpeedX=0.0
CurrentSpeedY=0.0
CurrentRotate=0.0
def moveX(speed):
	global ControllerFrequecny
	global ACC
	if speed>0:
		cmd.linear.x+=ACC/ControllerFrequecny
		if(cmd.linear.x>=speed):
			cmd.linear.x=speed
			print 'MAX SPEED'
			man.say("MAX SPEED")
		if cmd.linear.x<=speed:
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(1.0/ControllerFrequecny)
		print "move forward!"
	elif speed<0:
		cmd.linear.x-=ACC/ControllerFrequecny
		if(cmd.linear.x<=speed):
			cmd.linear.x=speed
			print 'MAX SPEED'
			man.say("max speed")
		if(cmd.linear.x>speed):
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(1.0/ControllerFrequecny)
		print "move back!"
	else:
		return
def moveY(speed):
	global ControllerFrequecny
	global ACC
	if speed>0:
		cmd.linear.y+=ACC/ControllerFrequecny
		if(cmd.linear.y>=speed):
			cmd.linear.y=speed
			print 'MAX SPEED'
			man.say("max speed")
		cmd.linear.x=0.0
		cmd.angular.z=0.0
		pub.publish(cmd)
		time.sleep(1.0/ControllerFrequecny)
		print "move left!"
	elif speed<0:
		cmd.linear.y-=ACC/ControllerFrequecny
		if(cmd.linear.y<=speed):
			cmd.linear.y=speed
			print 'MAX SPEED'
		cmd.linear.x=0.0
		cmd.angular.z=0.0
		pub.publish(cmd)
		time.sleep(1.0/ControllerFrequecny)
		print "move right!"
	else:
		return
def stop_robot():
	global ControllerFrequecny
	global ACC
	global RotateAcc
	fx=False
	fy=False
	fz=False
	while True:
		if(cmd.linear.x>0):
			cmd.linear.x-=ACC/ControllerFrequecny
		elif(cmd.linear.x<0):
			cmd.linear.x+=ACC/ControllerFrequecny
		if(cmd.linear.y>0):
			cmd.linear.y-=ACC/ControllerFrequecny
		elif (cmd.linear.y<0):
			cmd.linear.y+=ACC/ControllerFrequecny
		if (cmd.angular.z>0):
			cmd.angular.z-=RotateAcc/ControllerFrequecny
		elif(cmd.angular.z<0):
			cmd.angular.z+=RotateAcc/ControllerFrequecny
		if(abs(cmd.linear.x)<(2*ACC/ControllerFrequecny)):
			cmd.linear.x=0.0
			fx=True
		if(abs(cmd.linear.y)<(2*ACC/ControllerFrequecny)):
			cmd.linear.y=0.0
			fy=True
		if(abs(cmd.angular.z)<(2*RotateAcc/ControllerFrequecny)):
			cmd.angular.z=0.0
			fz=True
		pub.publish(cmd)
		time.sleep(1.0/ControllerFrequecny)
		print 'stoping robot now'
		if fx and fy and fz:
			return
def rotateRobot(speed):
	global ControllerFrequecny
	global RotateAcc
	if speed>0:
		cmd.angular.z+=RotateAcc/ControllerFrequecny
		if(cmd.angular.z>=speed):
			cmd.angular.z=speed
			print 'MAX SPEED'
		cmd.linear.x=0.0
		cmd.angular.y=0.0
		pub.publish(cmd)
		time.sleep(1.0/ControllerFrequecny)
		print "rotate to left!"
	elif speed<0:
		cmd.angular.z-=RotateAcc/ControllerFrequecny
		if(cmd.angular.z<=speed):
			cmd.angular.z=speed
			print 'MAX SPEED'
		cmd.linear.x=0.0
		cmd.angular.y=0.0
		pub.publish(cmd)
		time.sleep(1.0/ControllerFrequecny)
		print "rotate to left!"
	else:
		return	
if __name__ == '__main__':
	rospy.init_node('teleop')
	rate = rospy.Rate(rospy.get_param('~hz', 1))  
	fd=sys.stdin.fileno()
	old_settings=termios.tcgetattr(fd)
	print "Reading form keybord"
	print """   i
j  k  l
   m"""
	print 'press Q to quit'
	while True:
		#old_settings[3]= old_settings[3] & ~termios.ICANON & ~termios.ECHO  
		try:
			tty.setraw(fd)
			ch=sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  
			#print 'error'
		if ch=='i':
			moveX(MAXSPEED)
		elif ch=='m':
			moveX(0-MAXSPEED)
		elif ch=='j':
			rotateRobot(MAXROTATESPEED)
		elif ch=='l':
			rotateRobot(-MAXROTATESPEED)
		elif ch=='u':
			rotateRobot(MAXROTATESPEED)
		elif ch=='o':
			rotateRobot(-MAXROTATESPEED)
		elif ch=='k':
			stop_robot()
		elif ch=='q':
			cmd.linear.x=0.0
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			print "shutdown!"
			break
		else:
			stop_robot()
			cmd.linear.x=0.0
			cmd.angular.z=0.0
		print "Reading form keybord"
		print """   i
j  k  l
   m"""
		print 'press Q to quit'
		#rate.sleep()

		
			
			
