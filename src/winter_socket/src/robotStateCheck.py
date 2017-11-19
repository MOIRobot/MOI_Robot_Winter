#!/usr/bin/env python   
# -*- coding:UTF-8 -*-  
from socket import *  
from time import ctime 

CMD=
"""
a: roscore on
b: roscore off
c: robot driver on(odom message)
d: robot driver off(odom message)
e: robot laser on
f: robot laser off
""" 
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
RobotAddress=None
while True:  
	try: 
		data, addr = udpSerSock.recvfrom(BUFSIZE)  
		#print 'Robot Online!'
		#print'received message from %s >> %s' % (addr, data)  
		#print "robot Online"
		#udpSerSock.sendto("Confirmed!",addr)  
		RobotAddress=addr
	except Exception:
		print 'Robot Off'
	if RobotAddress:
		data=raw_input(CMD+"please input control cmd:")
		udpSerSock.sendto(data,RobotAddress)
		

udpSerSock.close()  
