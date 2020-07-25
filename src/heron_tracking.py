#! /usr/bin/env python

"""


"""

import sys
import math
import rospy
import numpy as np
import random as rd
from nav_msgs.msg import Odometry
from heron_msgs.msg import Course
from go_thesis.msg import RangeOnly
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Vector3Stamped
from go_thesis.srv import HeronMotion, HeronMotionRequest

class Tracking:
	def __init__(self):
		# ASV
		self.asv_x = 0
		self.asv_y = 0
		self.asv_z = 0
		self.asv_yaw = 0
		# AUV
		self.auv_x = 7
		self.auv_y = -3
		self.auv_z = 0
		# Range
		self.range = 0
		# Particles
		self.particles_x = [0]*500
		self.particles_y = [0]*500

		# Goal
		self.goal_x = 0
		self.goal_y = 0
		self.goalReached = True
		
		self.pubCourse = rospy.Publisher('/cmd_course', Course, queue_size=1)
		sub_ro = rospy.Subscriber('/lbl_range', RangeOnly, self.ro_sub)
		sub_imu = rospy.Subscriber('/imu/rpy', Vector3Stamped, self.imu_sub)
		sub_auv = rospy.Subscriber('particle_filter/pose', Odometry, self.auv_sub)
		sub_asv = rospy.Subscriber('/odometry/gps', Odometry, self.asv_sub)
		sub_particles = rospy.Subscriber('/particle_filter/particles',PointCloud,self.particles_sub)

	def confidence_ellipse(self,x,y,n_std=3.0):
		cov = np.cov(x,y)
		pearson = cov[0,1]/np.sqrt(cov[0,0] * cov[1,1])
		# Using a special case to obtain the eigenvalues of this 2d dataset:
		ell_radius_x = np.sqrt(1 + pearson)
		ell_radius_y = np.sqrt(1 - pearson)
		
		# Calculating the standard deviation of x from the squareroot of the variance and multiplying with the given nb of standard deviations
		scale_x = np.sqrt(cov[0,0]) * n_std
		mean_x = np.mean(x)

		# Calculating the standard deviation of y ...
		scale_y = np.sqrt(cov[1,1]) * n_std
		mean_y = np.mean(y)

		lbda_1 = 3*math.sqrt((cov[0,0]+cov[1,1])/2+math.sqrt(math.pow((cov[0,0]-cov[1,1])/2,2)+math.pow(cov[0,1],2)))
		orientation = math.atan2(math.pow(lbda_1/3,2)-cov[0,0],cov[1,0])

		return orientation


	def control(self):
		rospy.wait_for_service('heron_controller')
		try:
			heron_controller = rospy.ServiceProxy('heron_controller',HeronMotion)
			cmd = heron_controller("STOP",0)
			print self.goalReached
			
			if self.goalReached == True:
				if math.sqrt(math.pow(self.asv_x-self.auv_x,2)+math.pow(self.asv_y-self.auv_y,2)) > 10:
					# Calculate the ellipse orientation.
					phi = self.confidence_ellipse(self.particles_x,self.particles_y)
					# Two positions are computed at 2.3m of the ellipse center.
					xp = self.auv_x + 2.5*math.cos(phi)
					yp = self.auv_y + 2.5*math.sin(phi)
					xm = self.auv_x - 2.5*math.cos(phi)
					ym = self.auv_y - 2.5*math.sin(phi)
					# The closest point is the one where the ASV will go.
					if math.sqrt(math.pow(xp-self.asv_x,2)+math.pow(yp-self.asv_y,2)) >= math.sqrt(math.pow(xm-self.asv_x,2)+math.pow(ym-self.asv_y,2)):
						self.goal_x = xm
						self.goal_y = ym
					else:
						self.goal_x = xp
						self.goal_y = yp
					self.goalReached = False

			else:
				# Compute the bearing
				bearing = math.atan2(self.goal_y-self.asv_y, self.goal_x-self.asv_x)
				# Take the fastest rotational direction
				if abs(bearing-self.asv_yaw) > 0.15:
					if bearing >= 0:
						if bearing < self.asv_yaw and bearing >= self.asv_yaw - math.pi:
							orientation = -1
						else:
							orientation = +1
					else:
						if bearing > self.asv_yaw and bearing <= self.asv_yaw + math.pi:
							orientation = +1
						else:
							orientation = -1
					# Turn accordingly to the actual orientation and the bearing
					if orientation > 0:
						cmd = heron_controller("ANTICLOCKWISE",0)
					else:
						cmd = heron_controller("CLOCKWISE",0)
				# If the direction is correct, move forward
				else:
					cmd = heron_controller("FORWARD",0)

			########################################
			if abs(self.asv_x-self.goal_x) < 0.5 and abs(self.asv_y-self.goal_y) < 0.5:
				cmd = heron_controller("STOP",0)
				self.goalReached = True
			########################################

			return cmd
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)


	def ro_sub(self,msg):
		self.range = math.sqrt(math.pow(msg.range,2) - math.pow(self.auv_z-self.asv_z,2))

		self.control()
	
	def imu_sub(self,msg):
		self.asv_yaw = msg.vector.z

	def auv_sub(self,msg):
		self.auv_x = msg.pose.pose.position.x
		self.auv_y = msg.pose.pose.position.y
		self.auv_z = msg.pose.pose.position.z
	
	def asv_sub(self,msg):
		self.asv_x = msg.pose.pose.position.x
		self.asv_y = msg.pose.pose.position.y
		self.asv_z = msg.pose.pose.position.z
		self.control()

	def particles_sub(self,msg):
		for i in range(500):
			self.particles_x[i] = msg.points[i].x
			self.particles_y[i] = msg.points[i].y

def main(args):
	rospy.init_node('heron_tracking',anonymous=True)
	rospy.loginfo("Starting heron_tracking.py")
	start = Tracking()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)