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
cmd = Twist()
pub = rospy.Publisher('cmd_vel',Twist,queue_size=20)


global CurrentSpeedX
global CurrentSpeedY
global CurrentRotate
global TIME
global ACC
TIME=0.1
ACC=0.1

CurrentSpeedX=0.0
CurrentSpeedY=0.0
CurrentRotate=0.0
def moveForward(speed):
	global CurrentSpeedX
	global TIME
	global ACC
	CurrentSpeedX=speed#表示当前的速度
	num=abs(int(speed/ACC))
	cmd.linear.x=0.0
	if speed>0:
		for i in range(num):
			cmd.linear.x+=ACC
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(TIME)
		print "move forward!"
	elif speed<0:
		for i in range(num):
			cmd.linear.x-=ACC
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(TIME)
		print "move forward!"	
	else:
		return
def moveLeft(speed):
	global CurrentSpeedY
	global TIME
	global ACC
	CurrentSpeedY=speed#表示当前的速度
	num=abs(int(speed/ACC))
	cmd.linear.x=0.0
	if speed>0:
		for i in range(num):
			cmd.linear.y+=ACC
			cmd.linear.x=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(TIME)
		print "move forward!"
	elif speed<0:
		for i in range(num):
			cmd.linear.y-=ACC
			cmd.linear.x=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			time.sleep(TIME)
		print "move forward!"	
	else:
		return

def stop_robot():
	global CurrentSpeedX
	global CurrentSpeedY
	global TIME
	global ACC
	speed=0.0
	if abs(CurrentSpeedX)>abs(CurrentSpeedY):
		speed=abs(CurrentSpeedX)
	else:
		speed=abs(CurrentSpeedY)
	if speed==0.0:
		return
	#print speed
	num=int(speed/ACC)
	#print num
	cmd.linear.x=CurrentSpeedX
	cmd.linear.y=CurrentSpeedY
	print (CurrentSpeedX/num)
	for i in range(num-1):
		cmd.linear.x-=(CurrentSpeedX/num)
		cmd.linear.y-=(CurrentSpeedY/num)
		cmd.angular.z=0.0
		pub.publish(cmd)
		time.sleep(TIME)
	cmd.linear.x=0.0
	cmd.linear.y=0.0
	cmd.angular.z=0.0
	pub.publish(cmd)
	
	CurrentSpeedX=0.0
	CurrentSpeedY=0.0

def rotateRobot(speed):
	global CurrentRotate
	global TIME
	CurrentRotate=speed#表示当前的速度
	num=abs(int(speed/0.1))
	cmd.linear.x=0.0
	cmd.linear.y=0.0
	cmd.linear.z=0.0
	if speed>0:
		for i in range(num):
			cmd.linear.y=0.0
			cmd.linear.x=0.0
			cmd.angular.z+=0.1
			pub.publish(cmd)
			time.sleep(TIME)
		print "move forward!"
	elif speed<0:
		for i in range(num):
			cmd.linear.y=0.0
			cmd.linear.x=0.0
			cmd.angular.z-=0.1
			pub.publish(cmd)
			time.sleep(TIME)
		print "move forward!"	
	else:
		return
def stoprotate():
	global CurrentRotate
	global TIME
	print CurrentRotate
	if CurrentRotate==0.0:
		return
	if  CurrentRotate>0.0:
		speed=CurrentRotate#表示当前的速度
		num=abs(int(speed/0.1))
		cmd.linear.x=0.0
		cmd.linear.y=0.0
		cmd.linear.z=0.0
		for i in range(num-1):
			cmd.linear.y=0.0
			cmd.linear.x=0.0
			cmd.angular.z-=0.1
			pub.publish(cmd)
			time.sleep(TIME)
		cmd.linear.x=0.0
		cmd.linear.y=0.0
		cmd.angular.z=0.0
		pub.publish(cmd)
	elif CurrentRotate<0.0:
		speed=CurrentRotate#表示当前的速度
		num=abs(int(speed/0.1))
		cmd.linear.x=0.0
		cmd.linear.y=0.0
		cmd.linear.z=0.0
		for i in range(num-1):
			cmd.linear.y=0.0
			cmd.linear.x=0.0
			cmd.angular.z+=0.1
			pub.publish(cmd)
			time.sleep(TIME)
		cmd.linear.x=0.0
		cmd.linear.y=0.0
		cmd.angular.z=0.0
		pub.publish(cmd)
	CurrentRotate=0.0
if __name__ == '__main__':
	rospy.init_node('teleop')
	rate = rospy.Rate(rospy.get_param('~hz', 1))  
	print "Reading form keybord"
	print """   i
j  k  l
   m"""
	print 'press Q to quit'
	while True:
		fd=sys.stdin.fileno()
		old_settings=termios.tcgetattr(fd)
		#old_settings[3]= old_settings[3] & ~termios.ICANON & ~termios.ECHO  
		try:
			tty.setraw(fd)
			ch=sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  
			#print 'error'
		if ch=='i':
			moveForward(0.3)
		elif ch=='m':
			moveForward(-0.3)
		elif ch=='j':
			moveLeft(0.3)
			print "turn left!"
		elif ch=='l':
			moveLeft(-0.3)
			print "turn right!"
		elif ch=='u':
			rotateRobot(-0.8)
			print "turn right!"
		elif ch=='o':
			rotateRobot(0.8)
			print "turn right!"
		elif ch=='k':
			stop_robot()
			stoprotate()
			print "stop motor!"
		elif ch=='q':
			cmd.linear.x=0.0
			cmd.linear.y=0.0
			cmd.angular.z=0.0
			pub.publish(cmd)
			print "shutdown!"
			break
		else:
			cmd.linear.x=0.0
			cmd.angular.z=0.0
		print "Reading form keybord"
		print """   i
j  k  l
   m"""
		print 'press Q to quit'
		#rate.sleep()

		
			
			
