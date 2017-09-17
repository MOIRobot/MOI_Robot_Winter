#!/usr/bin/env python
# -*- coding: utf-8 -*-

#通过接受/robot_state 下机器人的状态来发出状态声音
#

import roslib; roslib.load_manifest('winter_voice') 
#添加此句给pythonimport 时 寻找其他的文件用来
import sys
spath=sys.path[0]
snum=spath.rfind('/')
ssyspath=spath[0:snum]
sys.path.append(ssyspath)

import rospy
from std_msgs.msg import String
import os
class Voice:
	def __init__(self):
		rospy.init_node('voice_driver',anonymous=True)
		rospy.Subscriber("/robot_state",String,self.stateCallback)
		rospy.spin()
	def stateCallback(self,msg):
		if msg.data =="StartMove":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/StartMove.mp3")
			#os.system("play Voice/Sound/StartMove.mp3")
		elif msg.data =="StartMoveZ":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/StartMoveZ.mp3")
		elif msg.data =="GoalReached":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/GoalReached.mp3")
		elif msg.data =="GoalReachedZ":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/GoalReachedZ.mp3")
		elif msg.data =="Rotate":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/Rotate.mp3")
		elif msg.data =="RotateZ":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/RotateZ.mp3")
		elif msg.data =="MoveError":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/MoveError.mp3")
		elif msg.data =="MoveErrorZ":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/MoveErrorZ.mp3")
		elif msg.data =="RotateErrorZ":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/RotateErrorZ.mp3")
		elif msg.data =="RotateError":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/RotateError.mp3")
		elif msg.data =="Error":
			os.system("play ~/mecAGV/src/agv/src/Voice/Sound/Error.mp3")
		else:
			return 	
if __name__ == '__main__':
	Voice()
