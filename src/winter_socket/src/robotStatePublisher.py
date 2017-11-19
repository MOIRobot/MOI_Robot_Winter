#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *  
from time import ctime  
import os
import subprocess
import sys
import time
import commands

#获取当前文件目录
cpath=sys.path[0]
roscoreNode=None
robotdriverNode=None
robotlaserNode=None

CMD=
"""
a: roscore on
b: roscore off
c: robot driver on(odom message)
d: robot driver off(odom message)
e: robot laser on
f: robot laser off
""" 

def pkill(msg):
	cmd="pgrep "+msg
	p = commands.getoutput(cmd)
	commands.getoutput("kill %s " % p)
def paraseCMD(msg):
	global roscoreNode
	if "a" in msg:
		print "get roscore start message"
		udpCliSock.sendto("roscore on",ADDR) 
		cmd=cpath+"/autoLoad.sh"
		roscoreNode=subprocess.Popen(cmd)		
	elif "b" in msg:
		print "get roscore start message"
		udpCliSock.sendto("roscore off",ADDR) 
		pkill("roscore")
	elif "c" in msg:
		print "robot driver on(odom message)"
		udpCliSock.sendto("robot driver on(odom message)",ADDR) 
		cmd=cpath+"/robtodriver.sh"
		robotdriverNode=subprocess.Popen(cmd)
	elif "d" in msg:
		print "robot driver off(odom message)"
		udpCliSock.sendto("robot driver off(odom message)",ADDR) 
		pkill("base_control")	
	elif "e" in msg:
		print "robot laser on"
		udpCliSock.sendto("robot laser on",ADDR) 
		cmd=cpath+"/ls_laser.sh"
		robotlaserNode=subprocess.Popen(cmd)
	elif "f" in msg:
		print "robot laser off"
		udpCliSock.sendto("robot laser off",ADDR) 
		pkill("lslidar_n301_dr")
		pkill("lslidar_n301_de")	
def SendStatus():
	data="Robot Online!"
	udpCliSock.sendto(data,ADDR) 


HOST = '<broadcast>'  
PORT = 21567  
BUFSIZE = 1024
ADDR = (HOST, PORT)  
udpCliSock = socket(AF_INET, SOCK_DGRAM)
#设置阻塞
udpCliSock.setblocking(1)
#设置超时时间
udpCliSock.settimeout(2)
udpCliSock.bind(('', 0))  
udpCliSock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
  
while True:  
	SendStatus()
	try:
		data,ADDR = udpCliSock.recvfrom(BUFSIZE)
		paraseCMD(data)
	except Exception :
		print "Waiting Message"
		continue
udpCliSock.close()  
