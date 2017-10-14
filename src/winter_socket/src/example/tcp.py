import socket   
import time   
address = ('183.230.40.33',80)
HOST='183.230.40.33'
PORT=80
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
#msg="POST /devices/4070770/datapoints?type=5 HTTP/1.1"+"\r\n"+"Host:api.heclouds.com"+"\r\n"+"api-key:RwvM=JUR4ylFW6fpM9T3ZIitat4="+"\r\n"+"content-length:8"+"\r\n"+"\r\n"+",;air,50"
#print msg
datas2="GET /devices/4070770/datastreams?datastream_ids=open,music HTTP/1.1"+"\r\n"+"Host:api.heclouds.com"+"\r\n"+"api-key:RwvM=JUR4ylFW6fpM9T3ZIitat4="+"\r\n"+"\r\n"+"\r\n"
datas2="GET /devices/4070770/datastreams/open HTTP/1.1"+"\r\n"+"Host:api.heclouds.com"+"\r\n"+"api-key:RwvM=JUR4ylFW6fpM9T3ZIitat4="+"\r\n"+"\r\n"+"\r\n"
datas="GET /devices/4093170/datastreams/Reg_id HTTP/1.1"+"\r\n"+"Host:api.heclouds.com"+"\r\n"+"api-key:9dItUj6=WCjWwWbp3=c1ZPlO=dA="+"\r\n"+"\r\n"+"\r\n"
datas4="GET /devices/4093170/datastreams HTTP/1.1"+"\r\n"+"Host:api.heclouds.com"+"\r\n"+"api-key:9dItUj6=WCjWwWbp3=c1ZPlO=dA="+"\r\n"+"\r\n"+"\r\n"
s.sendall(datas4)
data=s.recv(1024)
print data
'''
while True:
	s.sendall(datas)
	print datas
	data=s.recv(1024)
	print data
	time.sleep(2)
s.close()
'''
