#!/usr/bin/env python
#coding=utf-8

import roslib; roslib.load_manifest('winter_simulation')

import rospy
import tf
import sys, select, termios, tty
from geometry_msgs.msg  import Twist, Quaternion, TransformStamped
from sensor_msgs.msg    import Range
from tf.transformations import quaternion_from_euler
import time
import thread
class PublishUnltro():
    def getDistance(self):
		while True:
			self.dist=input("plase input dis:") 
			print "you have input:"+str(self.dist)
    def Publish(self):
		#初始化节点
        rospy.init_node("talkerUltraSound", anonymous = True)        
        #超声波数据发布节点
        ultrasound_pub = rospy.Publisher("UltraSoundPublisher", Range, queue_size = 20)
        
        ran_broadcaster = tf.TransformBroadcaster()
        
        rate=rospy.Rate(10)
        #print "Input distance('+' is the end of the number):"
        while not rospy.is_shutdown():
			#ch=self.getKey()
			#self.addChar(ch)
			ran_quat = Quaternion()
			ran_quat = quaternion_from_euler(0, 0, 0)	
			#发布TF关系
			ran_broadcaster.sendTransform((0.2,0.0,0.2),ran_quat,rospy.Time.now(),"ultrasound","base_link")
			#定义一个超声波消息
			ran = Range()
			ran.header.stamp = rospy.Time.now()
			ran.header.frame_id = "/ultrasound"
			ran.field_of_view = 1;
			ran.min_range = 0;
			ran.max_range = 5;
			ran.range = self.dist;
			ultrasound_pub.publish(ran)
			rate.sleep()		
    def __init__(self):
		self.dist=0.5
		thread.start_new_thread(self.getDistance, ())
		self.Publish()
        

if __name__=="__main__":
     PublishUnltro()
