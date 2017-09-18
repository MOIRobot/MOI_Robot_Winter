#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,PoseStamped
from nav_msgs.msg import Path
import PyKDL
def PathCallback(path):
	i=1
	lastGD=GD=quat_to_angle(path.poses[0].pose.orientation)
	newPath=[]
	
	newPath.append(path.poses[0])
	
	while i<len(path.poses):
		
		GD=quat_to_angle(path.poses[i].pose.orientation)
		if(abs(GD)-abs(lastGD))>0.175:
			newPath.append(path.poses[i])
			lastGD=GD
		i+=1
	return newPath
def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]	
if __name__ == '__main__':
	rospy.init_node('pathlistener', anonymous=False)
	rospy.Subscriber('/planner/planner/plan',Path,PathCallback)
	rospy.spin()
