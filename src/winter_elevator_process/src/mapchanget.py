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
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from math import radians, copysign, sqrt, pow, pi,cos
import tf
import math
import PyKDL

from std_msgs.msg import String

global doorState


rospy.init_node('laser_check', anonymous=False)



mapchange=rospy.Publisher('/change_map',UInt8MultiArray,queue_size=1)
r = rospy.Rate(20)


mapchangmsg=UInt8MultiArray()
while not rospy.is_shutdown():
	a=input("choose map0 or map1:")
	if a==1:
		data=[1,0]
	if a==0:
		data=[0,0]
	mapchangmsg.data=data
	mapchange.publish(mapchangmsg)
	time.sleep(1)
