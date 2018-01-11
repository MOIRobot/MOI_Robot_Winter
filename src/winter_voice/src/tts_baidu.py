#!/usr/bin/python
# -*- coding: UTF-8 -*-
'''
python 使用百度tts库合成语音文件 ，首先要安装好play命令
'''
import os
import sys, locale
'''
App ID: 8919993

API Key: 5X1XYGfOhRVND38ETHTaoeif

Secret Key: 6989b7eda9aaff924a93afad746ef9c1
'''
class Speak:
	def __init__ (self,message,times,launguage,isRemove):
		'''
		传入参数　消息　次数　是否清除合成的语音文件
		'''
		self.tts(message,launguage)
		#说几次
		for i in range(0,times):
			self.say()
		if isRemove:
			self.over()
	def tts(self,message,launguage):
		try:
			import requests
		except:
			print "请下载python-requests模块后使用..."
			exit(-1)
		import urllib
		s = requests.Session()
		mes=''
		if launguage is "zh":
			mes="http://tts.baidu.com/text2audio?lan=zh&pid=101&vol=9&ie=UTF-8&text="
		else:
			mes="http://tts.baidu.com/text2audio?lan=en&pid=101&vol=9&ie=UTF-8&text="
		s.get(mes+ urllib.quote(message))
		res = s.get(mes+ urllib.quote(message)).content
		f = open("tts-temp.mp3", "w")
		f.write(res)
		f.close()
	def say(self):
		os.system("play tts-temp.mp3")
	def play(music):
		os.system("play "+music)
man=Speak("等待电梯打开",1,"zh",True)
def over(self):
	os.system("rm tts-temp.mp3")
