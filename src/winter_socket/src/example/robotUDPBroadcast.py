#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *  
import time
HOST = '<broadcast>'  
PORT = 21567  
BUFSIZE = 20 
ADDR = (HOST, PORT)  
udpCliSock = socket(AF_INET, SOCK_DGRAM)
#设置阻塞
udpCliSock.setblocking(1)
#设置超时时间
udpCliSock.settimeout(0.5)
udpCliSock.bind(('', 0))  
udpCliSock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)  
while True:  
	data="Robot Online!"
	print "sending -> %s"%data  
	udpCliSock.sendto(data,ADDR)  
	try:
		data,ADDR = udpCliSock.recvfrom(BUFSIZE)
		if  data:  
			print data
		time.sleep(5)
	except Exception :
		continue
udpCliSock.close()  
