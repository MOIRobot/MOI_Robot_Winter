import socket   
import time   
address = ('jjfaedp.hedevice.com',876)
HOST='jjfaedp.hedevice.com'
PORT=876
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
msg="POST /devices/4093215/datapoints?type=5 HTTP/1.1"+"\r\n"+"Host:jjfaedp.hedevice.com"+"\r\n"+"api-key:YOgqd88cyq42E9rVNqOdDuvn8bw="+"\r\n"+"content-length:8"+"\r\n"+"\r\n"+",;open,0"
print msg
s.sendall(msg)
data=s.recv(1024)
print data
s.close()

