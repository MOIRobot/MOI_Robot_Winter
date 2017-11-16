#!/usr/bin/env python   
# -*- coding:UTF-8 -*-  
from socket import *  
from time import ctime  
import os
import subprocess
import sys
from rosmaster.master_api import NUM_WORKERS
'''
HOST = '127.0.0.1'  
PORT = 21567  
BUFSIZE = 1024  
ADDR = (HOST,PORT)  
udpSerSock = socket(AF_INET, SOCK_DGRAM)
#设置阻塞
udpSerSock.setblocking(1)
#设置超时时间 8s
udpSerSock.settimeout(8)  
udpSerSock.bind(('',PORT))
RobotOnline=False
'''
print "hello ga"
'''
def paraseCMD(msg):
	if "roscore start" in msg:
	elif "roscore stop" in msg:
	elif "bringup start" in msg:	
	elif "bringup stop" in msg:	
	elif "laser start" in msg:
	elif "laser stop" in msg:
	elif "gmapping"	in msg:
	elif "navigation" in msg:
	else:
	
while True:  
	try: 
		data, addr = udpSerSock.recvfrom(BUFSIZE)  
		#print 'Robot Online!'
		#print'received message from %s >> %s' % (addr, data)  
		
		#udpSerSock.sendto("Confirmed!",addr)  
	except Exception:
		print 'Robot Off'

udpSerSock.close()  
'''
