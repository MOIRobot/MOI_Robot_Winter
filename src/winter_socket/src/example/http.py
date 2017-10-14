#!/usr/bin/env python
# -*- coding: utf-8 -*-    
import httplib  
import urllib  
import serial 
from Tkinter import *           # 导入 Tkinter 库
def sendhttp(data):
	try:
		#data = urllib.urlencode({'temperature': 20})
		headers = {"api-key": "RwvM=JUR4ylFW6fpM9T3ZIitat4="}
		httpClient=httplib.HTTPConnection('api.heclouds.com')
		httpClient.request('POST', '/devices/4070770/datapoints?type=5',data,headers)           
		response = httpClient.getresponse()
		print response.status
		print response.reason
		print response.read()
		httpClient.close()
	except Exception,e:
		print e
class MSerialPort:
	def __init__(self,port,buand,timeout):
		try:
			self.port=serial.Serial(port,buand)  
			self.port.timeout=timeout
			if not self.port.isOpen():
				self.port.open()
		except Exception,msg:
				raise Exception("串口出错")
		#thread.start_new_thread(self.read_data,())    
	def port_open(self):
		if not self.port.isOpen():
			self.port.open()  
	def port_close(self):
		self.port.close()  
	def send(self,data):
		self.port.write(data)  
	def readLine(self):
		data=self.port.readline()
		return data
	def read_data(self):
		MSG=''
		while True:
			data=''
			MSG=self.port.read(1)
			if MSG==',':
				while MSG!='#':
					data+=MSG
					MSG=self.port.read(1)
				print data
				sendhttp(data)

class MessageShow:
	def __init__(self,message,time):	
		
		#传入参数　消息内容　和这条消息显示的时间
		
		#消息显示的时间
		self.WaitTime=time
		self.root = Tk()                     # 创建窗口对象的背景色
		self.root.wm_title("消息")#title
		#居中显示
		self.root.resizable(False,False)
		self.root.update() # update window ,must do
		curWidth = self.root.winfo_reqwidth() # get current width
		curHeight = self.root.winfo_height() # get current height
		scnWidth,scnHeight = self.root.maxsize() # get screen width and height
		# now generate configuration information
		tmpcnf = '%dx%d+%d+%d'%(200,100,
		(scnWidth-curWidth)/2,(scnHeight-curHeight)/2)
		self.root.geometry(tmpcnf)
		
		#l1=Label(self.root,text=message,background="yellow")
		#font=("宋体", 12, "normal")
		l1=Label(self.root,text=message,foreground="red")
		l1.pack(side=TOP,expand=YES)
		b1=Button(self.root,text="退出",command=self.xinlabel)	
		b1['width']=10
		b1['height']=1
		b1.pack(side=BOTTOM,expand=YES)
		
		thread.start_new_thread(self.autoDestory,())
		    
		self.root.mainloop()                 # 进入消息循环
	def xinlabel(self):
		self.root.destroy()
	def autoDestory(self):
		time.sleep(self.WaitTime)
		self.root.destroy()
	
if __name__ == '__main__': 
	#serialC=MSerialPort('/dev/ttyUSB0',9600,1)
	root = Tk()                     # 创建窗口对象的背景色
	curWidth = root.winfo_reqwidth() # get current width
	curHeight = root.winfo_height() # get current height
	scnWidth,scnHeight =root.maxsize() # get screen width and height
		# now generate configuration information
	tmpcnf = '%dx%d+%d+%d'%(300,200,
	(scnWidth-curWidth)/2,(scnHeight-curHeight)/2)
	root.geometry(tmpcnf)
	b1=Button(root,text="发布消息到OneNet",command=publish).pack()
	b2=Button(root,text="退出",command=quit).pack()
	root.mainloop()                 # 进入消息循环
	
	  
	
