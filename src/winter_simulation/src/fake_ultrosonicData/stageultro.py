#!/usr/bin/env python
#coding=utf-8

import roslib; roslib.load_manifest('winter_simulation')

import rospy
import tf
from geometry_msgs.msg  import Twist, Quaternion, TransformStamped
from sensor_msgs.msg    import Range
from tf.transformations import quaternion_from_euler

class PublishUnltro():
    def __init__(self):
        #初始化节点
        rospy.init_node("talkerUltraSound", anonymous = True)        
        #超声波数据发布节点
        ultrasound_pub = rospy.Publisher("UltraSoundPublisher", Range, queue_size = 20)
        
        ran_broadcaster = tf.TransformBroadcaster()
        
        rate=rospy.Rate(20)
        
        while not rospy.is_shutdown():
			ran_quat = Quaternion()
			ran_quat = quaternion_from_euler(0, 0, 0)	
			#发布TF关系
			ran_broadcaster.sendTransform((0.6,0.00,0.1),ran_quat,rospy.Time.now(),"ultrasound","robot_0/base_link")
			#定义一个超声波消息
			ran = Range()
			ran.header.stamp = rospy.Time.now()
			ran.header.frame_id = "/ultrasound"
			ran.field_of_view = 1;
			ran.min_range = 0;
			ran.max_range = 5;
			ran.range = 1;
			ultrasound_pub.publish(ran)
			rate.sleep()
        

if __name__=="__main__":
     PublishUnltro()
