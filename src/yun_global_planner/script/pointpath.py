#!/usr/bin/env python
#coding=utf-8

"""
2017-4-29 by Bob

pointpath.py文件描述的是带有交互功能的位点。
其中主要定义了一个类：PointPath.
区分了两种位点，一种是管理位点，一个是其他位点。

"""
import rospy
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *

#这个类用于管理创建基于InteractiveMarker的waypoint
class PointPath(InteractiveMarker):
	
	#构造函数，初始化参考框架，名字，描述，是否是管理waypoint
	def __init__(self, frame_id, name, description, is_manager=False):
	
		#调用InteractiveMarker的初始化函数
		InteractiveMarker.__init__(self)
		
		self.header.frame_id = frame_id
		self.name = name
		self.description = description
		
		#定义一个Marker
		self.marker = Marker()
		self.marker.type = Marker.CYLINDER
		self.marker.scale.x = 0.2
		self.marker.scale.y = 0.2
		self.marker.scale.z = 0.4
		self.marker.pose.position.z = 0.2
		
		#管理waypoint的颜色不一样
		if is_manager:
			self.marker.color.r = 0.8
			self.marker.color.g = 0.0
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
		else:
			self.marker.color.r = 0.0
			self.marker.color.g = 0.8
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
			
		#创建一个InteractiveMarkerControl
		self.marker_control = InteractiveMarkerControl()
		self.marker_control.always_visible = True
		self.marker_control.orientation.w = 1.0
		self.marker_control.orientation.x = 0.0
		self.marker_control.orientation.y = 1.0
		self.marker_control.orientation.z = 0.0
		
		#给control指定具体的marker
		self.marker_control.markers.append(self.marker)
		self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
		
		#给InteractiveMarker添加控制
		self.controls.append(self.marker_control)
		
	#每接收到一个interaction就调用一次回调函数
	def processFeedback(self, feedback):

		#回调函数得的当前的位置姿态
		self.pose = feedback.pose
		#print self.pose
		
		#p = feedback.pose.position
		#print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)


if __name__=="__main__":
	
	#初始化节点
	rospy.init_node("Waypoint_Test")
	
	#创建waypoint
	waypoint = PointPath("map", "manager", "manager", True)
	
	#创建server
	server = InteractiveMarkerServer("path")
	
	#添加waypoint和回调函数
	server.insert(waypoint, waypoint.processFeedback)
	
	#应用改变
	server.applyChanges()
	
	rospy.spin()













