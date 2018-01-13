#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import tf
import geometry_msgs.msg
import PyKDL
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

rospy.init_node('tf_turtle')
listener = tf.TransformListener()

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
def get_odom():
	 while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			return Point(*trans),quat_to_angle(Quaternion(*rot))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
(position, rotation) = get_odom()
print position.x


	
