#!/usr/bin/env python

"""
Simulates the Long Baseline (LBL) range-only localisation between the heron and the desistek_saga.

Input: 	/desistek_saga/ground_truth_to_tf_desistek_saga/pose
		/pose_gt

Output: /lbl_range

"""
import rospy
import sys
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from go_thesis.msg import RangeOnly


class LongBaseline:
	def __init__(self):
		self.rate = 20				# refreshing rate = 20Hz
<<<<<<< HEAD
		self.transmissionPeriod = 0.5	# transmission period = 0.5 s
=======
		self.transmissionPeriod = 0.5	# transmission period = 1 s
>>>>>>> b8e9d2ec2eef1b879dab684de500e73664dea49e
		self.acousticSpeed = 1500	# speed of acoustic waves underwater: 1500 m/s

		self.desistek_pose = [0,0,0]
		self.heron_pose = [0,0,0]

		self.time = 0
		self.counter = 0
		self.queue = []
		self.beamDist = 0

		sub_desistek = rospy.Subscriber('/desistek_saga/ground_truth_to_tf_desistek_saga/pose', PoseStamped, self.desistek_sub)
		sub_heron = rospy.Subscriber('/pose_gt', Odometry, self.heron_sub)
		self.pubRange = rospy.Publisher("/lbl_range", RangeOnly, queue_size=1)

	# Get the desistek's position
	def desistek_sub(self,msg):	# 20 Hz
		self.desistek_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		self.counter += 1
		self.range()

	# Get the heron's position
	def heron_sub(self,msg):	# 20 Hz
		self.heron_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
		self.time = msg.header.stamp.secs + msg.header.stamp.nsecs*math.pow(10,-9)

	# Compute and publish the range between the 2 robots
	def range(self):
		# Send an acoustic beam every transmission time
		if self.counter >= self.rate*self.transmissionPeriod:
			self.counter = 0
			self.queue.append([self.time,self.heron_pose])	# The acoustic beam is added to the queue of previous beams. The time and position of ASV are saved.

		# If there is at least 1 beam sent: compute the range with the actual position of the AUV.
		if len(self.queue) > 0:
			dist = math.sqrt((self.queue[0][1][0] - self.desistek_pose[0])**2 + (self.queue[0][1][1] - self.desistek_pose[1])**2 + (self.queue[0][1][2] - self.desistek_pose[2])**2)
			self.beamDist += 1
			if dist >= self.beamDist/self.rate*self.acousticSpeed:	# Takes in count the propagation speed of the acoustic waves
				ro = RangeOnly()
				ro.time = self.queue[0][0]
				ro.range = dist
				ro.x = np.random.normal(self.queue[0][1][0],1)
				ro.y = np.random.normal(self.queue[0][1][1],1)
				#ro.x = self.queue[0][1][0]
				#ro.y = self.queue[0][1][1]
				ro.z = self.queue[0][1][2]
				del self.queue[0]
				self.beamDist = 0
				self.pubRange.publish(ro)

def main(args):
	rospy.init_node('lbl_comm',anonymous=True)
	rospy.loginfo("Starting lbl.py")
	lbl = LongBaseline()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)