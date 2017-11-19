#!/usr/bin/env python   
# -*- coding:UTF-8 -*-  
from socket import *  
from time import ctime  
import os
import subprocess
import sys
import time
import commands

def pkill(msg):
	cmd="pgrep "+msg
	p = commands.getoutput(cmd)
	commands.getoutput("kill %s " % p)

#获取当前文件目录
spath=sys.path[0]
snum=spath.rfind('/')
ssyspath=spath[0:snum]
sys.path.append(ssyspath)
sys.path.append(spath)

HOST = '127.0.0.1'  
PORT = 21567  
BUFSIZE = 1024  
ADDR = (HOST,PORT)  
udpSerSock = socket(AF_INET, SOCK_DGRAM)
#设置阻塞
udpSerSock.setblocking(1)
#设置超时时间 8s
udpSerSock.settimeout(1)  
udpSerSock.bind(('',PORT))
RobotOnline=False


global roscoreNode
roscoreNode=None
def paraseCMD(msg):
	global roscoreNode
	if "roscore start" in msg:
		print "get roscore start message"
		cmd=spath+"/autoLoad.sh"
		roscoreNode=subprocess.Popen(cmd)
		#roscoreNode=subprocess.Popen(cmd,stdout=subprocess.PIPE)
		#roscoreNode=subprocess.Popen(cmd,stdout=subprocess.PIPE)
		#t#ime.sleep(3)
		#(stdoutdata, stderrdata)=roscoreNode.communicate(input=None,timeout=5)
		#print stdoutdata
		#print stdoutdata
		
	elif "roscore stop" in msg:
		pkill("roscore")
		
	
while True:  
	global roscoreNode
	try: 
		data, addr = udpSerSock.recvfrom(BUFSIZE)  
		#print 'Robot Online!'
		print'received message from %s >> %s' % (addr, data)  
		
		#udpSerSock.sendto("Confirmed!",addr)  
		paraseCMD(data)
		
	except Exception:
		print 'time out'
		
	if roscoreNode !=None:
		if  roscoreNode.poll() is None:
			print "roscore exists"
		else:
			print "roscore stop"
		
udpSerSock.close()  
