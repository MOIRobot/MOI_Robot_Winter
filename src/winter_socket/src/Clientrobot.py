#!/usr/bin/env python   
# -*- coding:UTF-8 -*-  
from socket import *  
from time import ctime 
import thread
import time
import subprocess
import sys

CMD="""
***********CMD*************
a: roscore on
s: robot driver on(odom message)
d: robot laser on
f: robot gmapping on
g:robot navigation on
k:robot webserver on

q: roscore off
w: robot driver off(odom message)
e: robot laser off
r: robot gmapping off
t:robot navigation off
k:robot webserver off
m:set roscore on robot side for local ros node

***************************
""" 
HOST = '127.0.0.1'  
PORT = 21567  
BUFSIZE = 1024  
ADDR = (HOST,PORT)  
udpSerSock = socket(AF_INET, SOCK_DGRAM)
#设置阻塞
udpSerSock.setblocking(1.5)
#设置超时时间 8s
udpSerSock.settimeout(10)  
udpSerSock.bind(('',PORT))
global RobotAddress
global RES
global SHOW_FLAG
RobotOnline=False
RobotAddress=None
SHOW_FLAG=False

#获取当前文件目录
cpath=sys.path[0]
roscoreNode=None
robotdriverNode=None
robotlaserNode=None

def ReceviceData():
	while True:
		global RobotAddress
		global RES
		global SHOW_FLAG
		try: 
			data, addr = udpSerSock.recvfrom(BUFSIZE)  
			#print 'Robot Online!'
			#print'received message from %s >> %s' % (addr, data)  
			#print'\nreceived message: %s\n'%data 
			#print "robot Online"
			#udpSerSock.sendto("Confirmed!",addr)  
			RobotAddress=addr
			
			if SHOW_FLAG:
				print "result:\n%s"%data
				SHOW_FLAG=False
			
		except Exception:
			print 'Robot Off'
def getData():
	global RobotAddress
	try: 
		data, addr = udpSerSock.recvfrom(BUFSIZE)  
		#print'\nreceived message: %s\n'%data 
	except Exception:
		print 'Robot Off'	
thread.start_new_thread(ReceviceData, ())
while True:  
	#time.sleep(1)
	#ReceviceData()
	#getData()
	if RobotAddress !=None:
		data=raw_input(CMD+"please input control cmd:")
		udpSerSock.sendto(data,RobotAddress)
		SHOW_FLAG=True
		time.sleep(1)

udpSerSock.close()  
