import socket   
import time
#address = ('220.248.63.65',6666)
#HOST='220.248.63.65'
#address = ('192.168.1.106',6666)

HOST='223.104.5.225'
PORT=1234
address=(HOST,PORT)
s=None
for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC,socket.SOCK_STREAM, 0, socket.AI_PASSIVE):  
	af, socktype, proto, canonname, sa = res  
	try:
		s = socket.socket(af, socktype, proto)
	except socket.error, err_msg:
		 s = None 
		 continue
	try:
		s.connect(sa)
	except socket.error, msg:
		s.close()
		s=None
		continue
	break
if s is None:
	print 'error'
msg="POST /devices/4070770/datapoints?type=5 HTTP/1.1"+"\r\n"+"Host:api.heclouds.com"+"\r\n"+"api-key:RwvM=JUR4ylFW6fpM9T3ZIitat4="+"\r\n"+"content-length:7"+"\r\n"+"\r\n"+",;tem,9"
s.sendall(msg)
data=s.recv(1024)
s.close()
print data

