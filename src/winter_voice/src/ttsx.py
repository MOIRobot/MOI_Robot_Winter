#!/usr/bin/python
# -*- coding: UTF-8 -*-
import pyttsx
class TSpeak:
	def __init__(self):
		self.engine=pyttsx.init()
		self.engine.setProperty('rate', 120)
	def say(self,words):
		self.engine.say(words)
	def wait(self):
		self.engine.runAndWait()
class TSpeakOnce:
	def __init__(self,words):
		self.engine=pyttsx.init()
		self.engine.setProperty('rate', 150)
		self.engine.say(words)
		self.engine.runAndWait()
man=TSpeak()
man.say("Jobs is happy now")
man.say("hello,would you like some water")
man.wait()
