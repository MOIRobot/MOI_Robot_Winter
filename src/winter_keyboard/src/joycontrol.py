#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('winter_keyboard') 
import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import  os  
import sys, select, termios, tty
import time    
import thread
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
pub = rospy.Publisher('smooth_cmd_vel',Twist,queue_size=20)
man=TSpeak()
global CurrentSpeedX
global CurrentSpeedY
global CurrentRotate
global ACC
global RotateAcc
global MAXSPEED
global MAXROTATESPEED

MAXSPEED=0.4
MAXROTATESPEED=0.5
ACC=0.2
RotateAcc=0.2
#控制频率
global ControllerFrequecny
ControllerFrequecny=10

CurrentSpeedX=0.0
CurrentSpeedY=0.0
CurrentRotate=0.0


class TSpeak:
	def __init__(self):
		self.engine=pyttsx.init()
		self.engine.setProperty('rate', 120)
	def say(self,words):
		self.engine.say(words)
	def wait(self):
		self.engine.runAndWait()

man = TSpeak()

msg="""
Reading form keybord"
    i
j   k  l
    m
press Q to quit
"""
def moveX(speed,state):
	global ControllerFrequecny
	global ACC
	if speed>0:
		cmd.linear.x+=ACC/ControllerFrequecny
		if(cmd.linear.x>=speed):
			cmd.linear.x=speed
		if cmd.linear.x<=speed:
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(1.0/ControllerFrequecny)
			#print 'MAX SPEED'
			#man.say('max speed')
			#man.wait()
		cmd.linear.y=0.0
		if state=='fleft':
			cmd.angular.z=0.4
		elif state=='fright':
			cmd.angular.z=-0.4
		else:
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
			cmd.linear.x-=2*ACC/ControllerFrequecny
		elif(cmd.linear.x<0):
			cmd.linear.x+=2*ACC/ControllerFrequecny
		if(cmd.linear.y>0):
			cmd.linear.y-=ACC/ControllerFrequecny
		elif (cmd.linear.y<0):
			cmd.linear.y+=ACC/ControllerFrequecny
		if (cmd.angular.z>0):
			cmd.angular.z-=3*RotateAcc/ControllerFrequecny
		elif(cmd.angular.z<0):
			cmd.angular.z+=3*RotateAcc/ControllerFrequecny
		if(abs(cmd.linear.x)<(3*ACC/ControllerFrequecny)):
			cmd.linear.x=0.0
			fx=True
		if(abs(cmd.linear.y)<(3*ACC/ControllerFrequecny)):
			cmd.linear.y=0.0
			fy=True
		if(abs(cmd.angular.z)<(3*RotateAcc/ControllerFrequecny)):
			cmd.angular.z=0.0
			fz=True
		pub.publish(cmd)
		time.sleep(1.0/ControllerFrequecny)
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
global currentState
currentState='n'

def callback(msg):
	print 'herer'
	global SPEED
	global currentState
	if msg.buttons[0]==1:
		currentState='rUp'
		if msg.axes[0]==1.0:
			currentState='fleft'
		if msg.axes[0]==-1.0:
			currentState='fright'
	elif msg.buttons[1]==1:
		currentState='rRight'
	elif msg.buttons[2]==1:
		currentState='rDown'
	elif msg.buttons[3]==1:
		currentState='rLeft'
	elif msg.buttons[7]==1:
		currentState='R1'
	elif msg.buttons[5]==1:
		currentState='R2'
	elif msg.buttons[4]==1:
		currentState='L2'
	elif msg.buttons[6]==1:
		currentState='L1'
	elif msg.axes[0]==1.0:
		currentState='lLeft'
	elif msg.axes[1]==1.0:
		currentState='lUp'
	elif msg.axes[0]==-1.0:
		currentState='lRight'
	elif msg.axes[1]==-1.0:
		currentState='lDown'
	else:
		currentState='n'
if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('teleopjoy')
	rospy.Subscriber("/joy",Joy,callback)
	rate = rospy.Rate(rospy.get_param('~hz', 1))  
	while not rospy.is_shutdown():
		global currentState
		if currentState=='rUp':
			moveX(MAXSPEED,currentState)
		elif currentState== 'rDown':
			moveX(0-MAXSPEED,currentState)
		elif currentState=='rLeft':
			rotateRobot(MAXROTATESPEED)
		elif currentState=='rRight':
			rotateRobot(0-MAXROTATESPEED)
		elif currentState=='fleft' or currentState=='fright':
			moveX(MAXSPEED,currentState);
		else:
			stop_robot()
			
			
