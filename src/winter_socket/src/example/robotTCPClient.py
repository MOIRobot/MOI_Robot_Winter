#!/usr/bin/env python  
from socket import *  
      
import socket   
import time   
import socket  
  
address = ('localhost' , 31500)  
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
s.connect(address)  
s.send('hihi')    
data = s.recv(512)  
print 'the data received is',data  

  
s.close()  
