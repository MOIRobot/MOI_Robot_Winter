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
<<<<<<< HEAD
pub = rospy.Publisher('cmd_vel',Twist,queue_size=20)
=======
pub = rospy.Publisher('/winter_controller/cmd_vel',Twist,queue_size=20)
>>>>>>> e32ec2a7b903f8597fdda949196191462bef90d0

def moveForward(speed):
	cmd.linear.x=speed
	cmd.linear.y=0.0
	cmd.angular.z=0.0
	pub.publish(cmd)
def moveLeft(speed):
	cmd.linear.y=speed
	cmd.linear.x=0.0
	cmd.angular.z=0.0
	pub.publish(cmd)
	pub.publish(cmd)
def stop_robot():
	cmd.linear.x=0.0
	cmd.linear.y=0.0
	cmd.angular.z=0.0
	pub.publish(cmd)
	pub.publish(cmd)
def stoprotate():
	cmd.linear.x=0.0
	cmd.linear.y=0.0
	cmd.angular.z=0.0
	pub.publish(cmd)
	pub.publish(cmd)
def rotateRobot(speed):
	cmd.linear.x=0.0
	cmd.linear.y=0.0
	cmd.angular.z=speed
	pub.publish(cmd)
	pub.publish(cmd)

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
<<<<<<< HEAD
			moveForward(0.8)
		elif ch=='m':
			moveForward(-0.8)
=======
			moveForward(0.3)
		elif ch=='m':
			moveForward(-0.3)
>>>>>>> e32ec2a7b903f8597fdda949196191462bef90d0
		elif ch=='j':
			moveLeft(0.8)
			print "turn left!"
		elif ch=='l':
			moveLeft(-0.8)
			print "turn right!"
		elif ch=='u':
<<<<<<< HEAD
			rotateRobot(0.6)
			print "turn right!"
		elif ch=='o':
			rotateRobot(-0.6)
=======
			rotateRobot(0.5)
			print "turn right!"
		elif ch=='o':
			rotateRobot(-0.5)
>>>>>>> e32ec2a7b903f8597fdda949196191462bef90d0
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

		
			
			
