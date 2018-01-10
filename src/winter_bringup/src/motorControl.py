#!/usr/bin/env python
# -*- coding: utf-8 -*-

from mserial import MSerialPort
import sys
TEST=False
if not TEST:
	portname='/dev/ttyUSB0'
	if len(sys.argv)>1:
		com1=sys.argv[1]
		portname='/dev/tty'+com1
	serialport=portname
	baundrate=115200
	mSerial=MSerialPort(serialport,baundrate,2)
def print16(arr):
	for i in arr:
		try:
			print('%#x'%i)
		except:
			print('%#x'%ord(i))
def arrToStr(arr):
	str1=''
	for num in arr:
		try:
			str1+=chr(num)
		except:
			str1+=num
	return str1
global ls
global rs
ls=0
rs=0
def MotorSetSpeed(a,b):
	global ls
	global rs
	right_wheel_velocity = rs;
	leftWheel_velocity=ls;
	WL_H = ((leftWheel_velocity) >> 8) & 0xFF;
	WL_L = leftWheel_velocity & 0xFF;
	WR_H = ((right_wheel_velocity) >> 8) & 0xFF;
	WR_L = (right_wheel_velocity) & 0xFF;
	SUM = (((WL_H + WL_L + WR_H + WR_L) & 0x3F) + 0x30);
	cmd=[0x54,0x45,0x07,'S','S',WL_H,WL_L,WR_H,WR_L,SUM,'#']
	mSerial.send(arrToStr(cmd))
#-----------------------------------------------------------
#-----------------------------------------------------------
		
if __name__ == '__main__':
	while True:
		global ls
		global rs
		a=input("Input:")
		if a==1:
			ls=input("Left Motor Speed:")
			MotorSetSpeed(0,0)
		elif a==2:
			rs=input("Right Motor Speed:")
			MotorSetSpeed(0,0)
	
