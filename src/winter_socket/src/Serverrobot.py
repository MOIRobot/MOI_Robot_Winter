#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *  
from time import ctime  
import os
import subprocess
import sys
import time
import commands



HOST = '<broadcast>'  
PORT = 21567  
BUFSIZE = 1024
ADDR = (HOST, PORT)  
udpCliSock = socket(AF_INET, SOCK_DGRAM)
#设置阻塞
udpCliSock.setblocking(1)
#设置超时时间
udpCliSock.settimeout(1)
udpCliSock.bind(('', 0))  
udpCliSock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
  


#获取当前文件目录
cpath=sys.path[0]+"/robot_sh"
roscoreNode=None
robotdriverNode=None
robotlaserNode=None
robotgmappingNode=None
robotnavigationNode=None
robotjoyNode=None

def pkill(msg):
	cmd="pgrep "+msg
	p = commands.getoutput(cmd)
	commands.getoutput("kill %s " % p)
def paraseCMD(msg):
	global roscoreNode
	global ADDR
	if "a" in msg:
		print "get roscore start message"
		udpCliSock.sendto("roscore on",ADDR) 
		cmd=cpath+"/autoLoad.sh"
		roscoreNode=subprocess.Popen(cmd)		
	elif "q" in msg:
		print "get roscore off message"
		udpCliSock.sendto("roscore off",ADDR) 
		pkill("roscore")
	elif "s" in msg:
		print "robot driver on(odom message)"
		udpCliSock.sendto("robot driver on(odom message)",ADDR) 
		cmd=cpath+"/robtodriver.sh"
		robotdriverNode=subprocess.Popen(cmd)
	elif "w" in msg:
		print "robot driver off(odom message)"
		udpCliSock.sendto("robot driver off(odom message)",ADDR) 
		pkill("base_control")	
	elif "d" in msg:
		print "robot laser on"
		udpCliSock.sendto("robot laser on",ADDR) 
		cmd=cpath+"/ls_laser.sh"
		robotlaserNode=subprocess.Popen(cmd)
	elif "e" in msg:
		print "robot laser off"
		udpCliSock.sendto("robot laser off",ADDR) 
		pkill("lslidar_n301_dr")
		pkill("lslidar_n301_de")
	elif "f" in msg:
		print "robot gmapping on"
		udpCliSock.sendto("robot gmapiing on",ADDR) 
		cmd=cpath+"/gmapping_core.sh"
		robotgmappingNode=subprocess.Popen(cmd)
	elif "r" in msg:
		print "robot gmapping off"
		udpCliSock.sendto("robot gmapping off",ADDR) 
		pkill("state_publisher")
		pkill("slam_gmapping")
	elif "g" in msg:
		print "robot navigation on"
		udpCliSock.sendto("robot navigation on",ADDR) 
		cmd=cpath+"/navigation_core.sh"
		robotnavigationNode=subprocess.Popen(cmd)
	elif "t" in msg:
		print "robot navigation off"
		udpCliSock.sendto("robot navigation off",ADDR) 
		pkill("move_base")
		pkill("state_publisher")
		pkill("map_server")
		pkill("amcl")
	elif "k" in msg:
		print "robot webserver on"
		udpCliSock.sendto("robot webserver on",ADDR) 
		cmd=cpath+"/webserver.sh"
		robotnavigationNode=subprocess.Popen(cmd)
	elif "i" in msg:
		print "robot webserver off"
		udpCliSock.sendto("robot webserver off",ADDR) 
		pkill("webserver.sh")
	elif "j" in msg:
		print "robot joycontrol on"
		udpCliSock.sendto("robot joycontrol on",ADDR) 
		cmd=cpath+"/joycontrol.sh"
		robotjoyNode=subprocess.Popen(cmd)
	elif "u" in msg:
		print "robot joycontrol off"
		udpCliSock.sendto("robot joycontrol off",ADDR) 
		commands.getoutput("rosnode kill joy_node")
		commands.getoutput("rosnode kill joycontrol")
def SendStatus():
	resultMessage="Robot Online!\n"
	if roscoreNode !=None:
		if  roscoreNode.poll() is None:
			resultMessage=resultMessage+"roscore on\n"
	if robotdriverNode !=None:
		if  robotdriverNode.poll() is None:
			resultMessage=resultMessage+"robot driver on\n"
	if robotlaserNode !=None:
		if  robotlaserNode.poll() is None:
			resultMessage=resultMessage+"robot laser on\n"
	if robotgmappingNode !=None:
		if  robotgmappingNode.poll() is None:
			resultMessage=resultMessage+"robot gmapping on\n"
	if robotnavigationNode !=None:
		if  robotnavigationNode.poll() is None:
			resultMessage=resultMessage+"robot navigation on\n"
	#print resultMessage
	try:
		udpCliSock.sendto(resultMessage,ADDR)
	except Exception:
		return
while True:  
	SendStatus()
	try:
		data,ADDR = udpCliSock.recvfrom(BUFSIZE)
		paraseCMD(data)
	except Exception :
		#print "Waiting Message"
		continue
udpCliSock.close()  
