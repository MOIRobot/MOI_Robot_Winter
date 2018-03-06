#!/usr/bin/env python 
# -*- coding: utf-8 -*-
#　stage 中机器人运动的驱动文件
import roslib; roslib.load_manifest('agv') 
import rospy
from geometry_msgs.msg import Twist 
from std_msgs.msg import String
import  os  
import  sys
import  tty, termios
import time
    
cmd1 = Twist()
cmd2 = Twist()
cmd3 = Twist()
cmd4 = Twist()
cmd5 = Twist()
cmd6 = Twist()
pub1 = rospy.Publisher('robot_1/cmd_vel',Twist,queue_size=20)
pub2 = rospy.Publisher('robot_2/cmd_vel',Twist,queue_size=20)
pub3 = rospy.Publisher('robot_3/cmd_vel',Twist,queue_size=20)
pub4 = rospy.Publisher('robot_4/cmd_vel',Twist,queue_size=20)
pub5 = rospy.Publisher('robot_5/cmd_vel',Twist,queue_size=20)
pub6 = rospy.Publisher('robot_6/cmd_vel',Twist,queue_size=20)

if __name__ == '__main__':
	rospy.init_node('teleop')
	rate = rospy.Rate(rospy.get_param('~hz', 10))  
	i=0
	while True:
		rate.sleep()
		i+=1
		pub1.publish(cmd1)
		pub2.publish(cmd2)
		pub3.publish(cmd3)
		pub4.publish(cmd4)
		pub5.publish(cmd5)
		pub6.publish(cmd6)
		if i==1:
			cmd1.linear.y=1
			cmd2.linear.x=0.7
			cmd3.linear.x=1
			cmd3.linear.y=1
			cmd4.linear.x=1.3
			cmd5.linear.x=10
			cmd5.angular.z=1.0
			cmd6.linear.x=-1
			cmd6.linear.y=-1
		elif i==30:
			cmd1.linear.y=-1
			cmd2.linear.x=-0.7
			cmd3.linear.x=-1
			cmd3.linear.y=-1
			cmd4.linear.x=-1.3
		elif i==50:
			cmd6.linear.x=1
			cmd6.linear.y=1
		elif i==70:
			cmd1.linear.y=1
			cmd2.linear.x=0.7
			cmd3.linear.x=1
			cmd3.linear.y=1
			cmd4.linear.x=1.3
		elif i==90:
			i=0
