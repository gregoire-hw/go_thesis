#! /usr/bin/env python

import rospy
import math
import numpy as np
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class save:
	def __init__(self):
		self.gt_x = 0
		self.gt_y = 0
		self.gt_z = 0
			
		sub_dvl = rospy.Subscriber('/desistek_saga/dvl/position',Odometry,self.dvl_sub)
		sub_ekf = rospy.Subscriber('/pose/ekf',Odometry,self.ekf_sub)
		sub_desistek = rospy.Subscriber('/desistek_saga/ground_truth_to_tf_desistek_saga/pose', PoseStamped, self.desistek_sub)

	def dvl_sub(self,msg):
		dvl_x = msg.pose.pose.position.x
		dvl_y = msg.pose.pose.position.y
		dvl_z = msg.pose.pose.position.z
		self.error("dvl",[dvl_x,dvl_y,dvl_z])

	def ekf_sub(self,msg):
		ekf_x = msg.pose.pose.position.x
		ekf_y = msg.pose.pose.position.y
		ekf_z = msg.pose.pose.position.z
		self.error("ekf",[ekf_x,ekf_y,ekf_z])

	def desistek_sub(self,msg):
		self.gt_x = msg.pose.position.x
		self.gt_y = msg.pose.position.y
		self.gt_z = msg.pose.position.z

	def error(self,name,position):
		error = math.sqrt(math.pow(self.gt_x-position[0],2)+math.pow(self.gt_y-position[1],2)+math.pow(self.gt_z-position[2],2))
		print name, error

def main(args):
	rospy.init_node('eval',anonymous=True)
	rospy.loginfo("Starting ekf_evaluation.py")
	lbl = save()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)