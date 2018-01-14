#!/usr/bin/env python
#coding=utf-8

"""
2017-5-6 by Bob
用于管理所有的位点，包括创建删除，加载保存位点

PointPathManager_3.py文件是第三个版本的路径文件，主要是
用于管理位点的创建，删除，加载，发布线路等等。
其中定义了一个类：PointPathManager
其主要想法是先创建好各种的位点，放置好位置，然后保存到硬盘。
接着在文件中定义好哪两个点之间会有线段连接，然后重新启动程序，
然后加载之前保存的位点，最后发布地定义的线路。

"""
import roslib; 
roslib.load_manifest("interactive_markers")
import rospy
import rospkg
import copy
import os
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from pointpath import PointPath
from geometry_msgs.msg import Point


#用于管理所有的位点，基于InteractiveMarkerServer类
class PointPathManager(InteractiveMarkerServer):

	def __init__(self, name, frame_id):
	
		#调用InteractiveMarkerServer
		InteractiveMarkerServer.__init__(self, name)
		
		#参考框架
		self.frame_id = frame_id
		
		#点的数目，以list的形式组织起来
		self.list_of_points = []
		
		#点的数目
		self.counter_points = 0
		
		self.pub = rospy.Publisher("/lines", Marker, queue_size=1)
		self.arrow = Marker()
		self.publishArrowFlag = False
		
		#创建菜单
		self.menu_handler = MenuHandler()
		self.menu_handler.insert("Create a Waypoint", callback=self.createWaypointCB)
		entry = self.menu_handler.insert("Delete Waypoint")
		self.menu_handler.insert("Delete last Waypoint", parent=entry, callback = self.deletelastWaypointCB)
		self.menu_handler.insert("Delete all Waypoints", parent=entry, callback = self.deleteallWaypointCB)
		self.menu_handler.insert("Load all Waypoints", callback=self.loadAllWaypointsCB)
		self.menu_handler.insert("Save all Waypoints", callback=self.saveAllWaypointsCB)
		self.menu_handler.insert("Publish Lines", callback=self.publishLinesCB)
		
		#创建地一个管理Point,管理节点不算入list中
		self.initial_point = PointPath(self.frame_id, "PointManager", "PointManager", True)
		
		#添加到服务器
		self.insert(self.initial_point, self.initial_point.processFeedback)
		
		#给一个Point添加菜单
		self.menu_handler.apply(self, self.initial_point.name)
		
		#应用变化
		self.applyChanges()
		
		#waypoints.txt的路径
		rp = rospkg.RosPack() #rospath
		self.points_file_path = os.path.join(rp.get_path("yun_global_planner"), "script", "waypoints.txt")
		lines_file_path = os.path.join(rp.get_path("yun_global_planner"), "script", "lines.txt")
		
		#定义lines
		#self.point_list = [0, 1,\
							#1, 2,\
							#2, 3,\
							#3, 4,\
							#4, 5,\
							#5, 6,\
							#7, 8,\
							#8, 9,\
							#9, 10,\
							#10, 11,\
							#11, 12,\
							#12, 13,\
							#13, 14]
		self.point_list = []
						
		#保存Lines
		try:
			#打开lines.txt
			file_lines = open(lines_file_path, "w")
		except IOError, e:
			rospy.logerr("Failed to Open %s : %s"%(file_lines, e))
			return
			
		for i in range(0, len(self.point_list), 2):
			#(P_0;P_1)
			line = "P_%d;P_%d\n"%(self.point_list[i],self.point_list[i+1])
			file_lines.write(line)
		
		#关闭文件
		file_lines.close()
		
		rospy.loginfo("Save %d lines"%(len(self.point_list)/2))
		
		
		#发布lines
		rate = rospy.Rate(30)
		
		while not rospy.is_shutdown():
			if self.publishArrowFlag == True:
				point_pairs = []
				#print "len of list of points %d"%len(self.list_of_points)
				for i in self.point_list:
					point_pairs.append(self.list_of_points[i])
				length = len(point_pairs)
				for i in range(0, length, 2):
					self.publishArrow(i, point_pairs[i], point_pairs[i+1])
			rate.sleep()
			
		
		
	def publishLinesCB(self, feedback):
		
		self.publishArrowFlag = True
	
	#发表所有的路线
	def publishArrow(self, arrow_id, start_point, end_point):
		#箭头
		self.arrow.header.frame_id = self.frame_id
		self.arrow.header.stamp = rospy.Time.now()
		self.arrow.ns = "arrow"
		self.arrow.id = arrow_id
		self.arrow.action = Marker.ADD
		self.arrow.type = Marker.ARROW
		self.arrow.scale.x = 0.04 #shaft diameter
		self.arrow.scale.y = 0.12 #head diameter
		self.arrow.scale.z = 0.3  #head length
		self.arrow.color.g = 1.0
		self.arrow.color.r = 1.0
		self.arrow.color.a = 0.8
		self.arrow.points = [0 for i in range(2)]
		self.start_point = Point()
		self.end_point = Point()
		self.start_point.x = start_point.pose.position.x
		self.start_point.y = start_point.pose.position.y
		self.end_point.x = end_point.pose.position.x
		self.end_point.y = end_point.pose.position.y
		self.arrow.points[0] = self.start_point
		self.arrow.points[1] = self.end_point
		self.pub.publish(self.arrow)
		
	#回调函数
	def createWaypointCB(self, feedback):
		print "createWaypointCB"
		
		#创建新的点
		new_point = PointPath(self.frame_id, "P_%d"%(self.counter_points), "P_%d"%(self.counter_points))
		
		if len(self.list_of_points) != 0:
			new_point.pose.position.x = self.list_of_points[self.counter_points - 1].pose.position.x
			new_point.pose.position.y = self.list_of_points[self.counter_points - 1].pose.position.y
		
		#新的点比之前的点在X轴上坐标大一
		new_point.pose.position.x = new_point.pose.position.x + 1.0
		
		#添加新的点到list
		self.list_of_points.append(new_point)
		
		#点的数目加1
		self.counter_points = self.counter_points + 1
		
		#添加点和回调函数到服务器
		self.insert(new_point, new_point.processFeedback)
		
		#应用菜单
		self.menu_handler.apply(self, new_point.name)
		
		#应用变化
		self.applyChanges()
		
	def deletelastWaypointCB(self, feedback):
		print "deletelastWaypointCB"
		
		#如果点数目不是0
		if self.counter_points > 0:
		
			#删除list中的点
			p = self.list_of_points.pop()
			self.counter_points = self.counter_points -1
			
			#擦除服务器中的点
			self.erase(p.name)
			
			#应用改变
			self.applyChanges()
			
			
	def deleteAllPoints(self):
		for i in range(len(self.list_of_points)):
			p = self.list_of_points.pop()
			self.counter_points = self.counter_points - 1
			self.erase(p.name)
			
		self.applyChanges()
			
		
	def deleteallWaypointCB(self, feedback):
		print "deleteallWaypointCB"
		
		self.deleteAllPoints()
		
	def loadAllWaypointsCB(self, feedback):
		print "loadAllWaypointsCB"
		#delete all loaded points
		if len(self.list_of_points) > 0:
			self.deleteAllPoints()
			
		try:
			file_points = open(self.points_file_path, "r")
		except IOError, e:
			rospy.logerr("Failed to Open %s : %s"%(self.points_file_path, e))
			return 
		
		num_of_loaded_points = 0
		
		#读取每一行的信息
		line = file_points.readline().replace("\n", "")
		
		#不是空行
		while line != "":
			a = line.split(";")
			
			if len(a) == 3:
				new_point = PointPath(self.frame_id, a[0], a[0])
				new_point.pose.position.x = float(a[1])
				new_point.pose.position.y = float(a[2])
				
				self.list_of_points.append(new_point)
				self.insert(new_point, new_point.processFeedback)
				self.menu_handler.apply(self, new_point.name)
				self.applyChanges()
				self.counter_points = self.counter_points + 1
				num_of_loaded_points = num_of_loaded_points + 1
			else:
				rospy.logerr(" Error processing line %s"%(line))
				
			#继续读取一行数据
			line = file_points.readline().replace('\n', '')
			
		#关闭文件
		file_points.close()
		
		rospy.loginfo("Loaded %d points"%(num_of_loaded_points))
		
		
	def saveAllWaypointsCB(self, feedback):
		print "saveAllWaypointsCB"
		try:
			#打开waypoints.txt
			file_points = open(self.points_file_path, "w")
		except IOError, e:
			rospy.logerr("Failed to Open %s : %s"%(self.points_file_path, e))
			return
			
		for i in self.list_of_points:
			#(name, x, y)
			line = "%s;%.3f;%.3f\n"%(i.name, i.pose.position.x, i.pose.position.y)
			file_points.write(line)
		
		#关闭文件
		file_points.close()
		
		rospy.loginfo("Save %d waypoints"%(len(self.list_of_points)))
	
	
		
if __name__=="__main__":
	
	rospy.init_node("PointPathManager")
	
	PointPathManager("agv", "map")
	
	rospy.spin()
	
	
	
	
	
	
	

