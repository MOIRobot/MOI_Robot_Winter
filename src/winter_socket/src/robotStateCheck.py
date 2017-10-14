#!/usr/bin/env python   
# -*- coding:UTF-8 -*-  
from socket import *  
from time import ctime  
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
while True:  
	try: 
		data, addr = udpSerSock.recvfrom(BUFSIZE)  
		print 'Robot Online!'
		print'received message from %s >> %s' % (addr, data)  
		udpSerSock.sendto("Confirmed!",addr)  
	except Exception:
		print 'Robot Off'

udpSerSock.close()  
