#!/usr/bin/env python   
# -*- coding:UTF-8 -*-  

import itchat
import requests
global AutoReplay
AutoReplay=True
def GetResponse(msg):
	apiUrl = 'http://www.tuling123.com/openapi/api'
	data={
	'key'    : '4d84bed3cb6ecc150fe9dc5493f321ea',
	'info'   : msg,
	'userid' : '2385909406@qq.com',
	}
	try:
		r = requests.post(apiUrl, data=data).json()
		return r.get('text')
	except:
		return 'error'
@itchat.msg_register(itchat.content.TEXT)
def print_content(msg):
	global AutoReplay
	user=msg['User']
	name=''
	try:
		name=user['RemarkName']
		print name
		print user['NickName']
	except:
		print "error"
	res=GetResponse(msg['Text'])
	itchat.send(u'自动回复关闭', 'filehelper')
	return res
itchat.auto_login(hotReload=True)
itchat.run()
