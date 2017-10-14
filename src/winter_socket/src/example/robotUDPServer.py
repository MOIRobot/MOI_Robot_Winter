#!/usr/bin/env python  
# -*- coding:UTF-8 -*-  
      
from socket import *  
from time import ctime  
      
HOST = '127.0.0.1'  
PORT = 21567  
BUFSIZE = 1024  
ADDR = (HOST,PORT)  
udpSerSock = socket(AF_INET, SOCK_DGRAM)  
udpSerSock.bind(ADDR)  
while True:  
	print 'wating for message...'  
	data, addr = udpSerSock.recvfrom(BUFSIZE)  
	udpSerSock.sendto('[%s] %s'%(ctime(),data),addr)  
	print '...received from and retuned to:',addr  
udpSerSock.close()  
