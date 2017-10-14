#!/usr/bin/env python
# -*- coding: utf-8 -*-
from socket import *
import time   
HOST='localhost' 
PORT=31500
ADDR=(HOST,PORT)
#创建TCP套接字  
tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
#listen()函数的参数表示最多允许同时接收的连接数，超过这个数目的连接会被拒绝掉  
tcpSerSock.listen(10)

while True:
	print 'waiting for connection...' 
	tcpCliSock, addr = tcpSerSock.accept()
	print '...connected from:', addr
	while True:
		data=tcpCliSock.recv(1024)
		print 'Received:'
		print data
		if not data:
			break;
		tcpCliSock.send('Confirmed!')
tcpCliSock.close()  
tcpSerSock.close()
