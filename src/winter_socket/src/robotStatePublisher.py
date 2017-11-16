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

def pkill(msg):
	cmd="pgrep "+msg
	p = commands.getoutput(cmd)
	commands.getoutput("kill %s " % p)
def paraseCMD(msg):
	global roscoreNode
	if "roscore start" in msg:
		print "get roscore start message"
		cmd=cpath+"/autoLoad.sh"
		roscoreNode=subprocess.Popen(cmd)		
	elif "roscore stop" in msg:
		pkill("roscore")
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
		continue
udpCliSock.close()  
