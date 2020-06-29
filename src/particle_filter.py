#! /usr/bin/env python

"""
Particle Filter Localisation for the AUV using dead reckoning + range-only localisation.

Input:	/desistek_saga/dvl
		/desistek_saga/imu
		/lbl_range	# 2Hz

Output: /pose/pf

"""

import rospy
import math
import numpy as np
import random as rd
import sys
from nav_msgs.msg import Odometry
from go_thesis.msg import RangeOnly
from go_thesis.msg import EkfMsg
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Imu
from uuv_sensor_ros_plugins_msgs.msg import DVL
import matplotlib.pyplot as plt

class ekf:
	def __init__(self):
		# Velocity estimation of the AUV from the DVL and IMU
		self.dvl_x = 0
		self.dvl_y = 0
		self.dvl_z = 0
		self.theta = 0
		self.depth = -80
		self.depth_cov = 0
		self.previous_depth_time = rospy.get_time()
		# Pose estimation of the ASV from the GPS
		self.asv_x = 0
		self.asv_y = 0
		self.asv_z = 0
		# Range between the 2 robots
		self.range = 0
		# Position obtained after filtering
		self.ekf_x = 0
		self.ekf_y = 0
		self.ekf_z = 0

		# Matrices
		self.nbParticles = 500	# Nb of particles = Nb of bearings
		self.initialisation = False

		self.predicted_state = np.zeros((self.nbParticles,3,1))
		self.predicted_covariance = np.zeros((self.nbParticles,3,3))

		self.corrected_state = np.zeros((self.nbParticles,3,1))
		self.corrected_covariance = np.zeros((self.nbParticles,3,3))

		self.matrix_A = np.array([[1.0,0.0,0.0],
			[0.0,1.0,0.0],
			[0.0,0.0,1.0]])

		self.matrix_I = np.array([[1.,0.,0.],
			[0.,1.,0.],
			[0.,0.,1.]])

		dvl_uncertainty = 0.05
		self.matrix_R = np.array([[dvl_uncertainty,0.,0.],
			[0.,dvl_uncertainty,0.],
			[0.,0.,0.03]])


		self.measurementUncertainty = 1.

		self.particleWeight = [0] * self.nbParticles

		# Other needed variables
		self.previous_time = 0
		self.previous_theta = 0
		self.gpsReceived = False
		self.dvlReceived = False
		self.gpsrate = 20
		self.counter = 0
		# Topics
		sub_dvl = rospy.Subscriber('/desistek_saga/dvl',DVL,self.dvl_sub)
		sub_imu = rospy.Subscriber('/desistek_saga/imu', Imu, self.imu_sub)
		sub_depth = rospy.Subscriber('/desistek_saga/depth',Odometry,self.depth_sub)
		sub_ro = rospy.Subscriber('/lbl_range',RangeOnly,self.ro_sub)
		self.pubPose = rospy.Publisher('/particle_filter/pose',Odometry,queue_size = 1)
		self.pubEkf = rospy.Publisher('/particle_filter/info',EkfMsg,queue_size = 1)
		self.pubParticles = rospy.Publisher('/particle_filter/particles',PointCloud,queue_size = 1)

	def filter(self):
		time = rospy.get_time()
		delta_time = time-self.previous_time

		# Starts at the 2nd value
		if self.previous_time != 0:

			# Initialisation Step:
			if self.initialisation == False and self.range != 0:
				for i in range(self.nbParticles):
					self.corrected_state[i][0] = self.range*math.cos((2*np.pi/self.nbParticles)*i) #+ self.asv_x
					self.corrected_state[i][1] = self.range*math.sin((2*np.pi/self.nbParticles)*i) #+ self.asv_y
					self.corrected_state[i][2] = self.theta
					self.particleWeight[i] = 1
				self.initialisation = True
			# End of Initialisation Step

			if self.initialisation == True:
				pcl = PointCloud()
				pcl.header.stamp.secs = int(time)
				pcl.header.stamp.nsecs = 1000000000*(time-int(time))
				pcl.header.frame_id = "world"

				for i in range(self.nbParticles):
					# Update matrices:
					# B
					matrix_B = np.array([[delta_time*math.cos(self.corrected_state[i][2]),delta_time*math.sin(self.corrected_state[i][2]),0.],
						[delta_time*math.sin(self.corrected_state[i][2]),-delta_time*math.cos(self.corrected_state[i][2]),0.],
						[0.,0.,delta_time]])
					# u 
					matrix_u = np.array([[self.dvl_x],
						[self.dvl_y],
						[(self.theta-self.corrected_state[i][2])/delta_time]])
					# G
					matrix_G = np.array([[1.,0.,delta_time*(-matrix_u[0]*math.sin(self.corrected_state[i][2])+matrix_u[1]*math.cos(self.corrected_state[i][2]))],
						[0.,1.,delta_time*(matrix_u[0]*math.cos(self.corrected_state[i][2])+matrix_u[1]*math.sin(self.corrected_state[i][2]))],
						[0.,0.,1.]])
					# Prediction Step:
					self.predicted_state[i] =  np.dot(self.matrix_A,self.corrected_state[i])+np.dot(matrix_B,matrix_u)
					self.predicted_covariance[i] = np.dot(matrix_G,np.dot(self.corrected_covariance[i],matrix_G.T)) + self.matrix_R
					# End of Prediction Step


					pcl.points.append(Point32(self.predicted_state[i][0],self.predicted_state[i][1],self.depth))


					inferred_range = math.sqrt(math.pow(self.asv_x-self.predicted_state[i][0],2)+math.pow(self.asv_y-self.predicted_state[i][1],2))
					# Particle Weighting:
					if abs(self.range - inferred_range) <= 3.0*self.measurementUncertainty:
						self.particleWeight[i] += 1
				

				self.pubParticles.publish(pcl)
				#############################################################################
				#########						RESAMPLING							#########
				#############################################################################
				particles = np.zeros((self.nbParticles,4))
				sumOfWeight = 0.
				sumOfWeightSquared = 0.
				a = 0
				# Sort the particles
				for i in range(self.nbParticles):
					particles[i][1] = self.predicted_state[i][0][0]
					particles[i][2] = self.predicted_state[i][1][0]
					particles[i][3] = self.predicted_state[i][2][0]
					sumOfWeight += self.particleWeight[i]


				for i in range(self.nbParticles):
					particles[i][0] = self.particleWeight[i]/sumOfWeight
					sumOfWeightSquared += math.pow(particles[i][0],2)


				particles = sorted(particles, key=lambda particles: particles[0])

				Neff = 1/sumOfWeightSquared
				#print Neff

				if Neff < self.nbParticles*0.998:
					c = [0]*self.nbParticles
					u = [0]*self.nbParticles

					c[0] = 0
					for i in range(self.nbParticles-1):
						c[i+1] = c[i] + particles[i+1][0]

					i = 0
					u[0] = rd.random()/self.nbParticles
					for j in range(self.nbParticles):
						u[j] = u[0]+float(j)/self.nbParticles
						while u[j] > c[i] and i < self.nbParticles-1:
							i+=1
						self.particleWeight[j] = particles[i][0]*self.nbParticles
						self.corrected_state[j][0] = np.random.normal(particles[i][1],0.2)
						self.corrected_state[j][1] = np.random.normal(particles[i][2],0.2)
						self.corrected_state[j][2] = particles[i][3]

				else:
					self.corrected_state = self.predicted_state


				avgX = 0
				avgY = 0
				for i in range(self.nbParticles):
					avgX += self.corrected_state[i][0]
					avgY += self.corrected_state[i][1]
				avgX = avgX/self.nbParticles
				avgY = avgY/self.nbParticles
				self.publish(avgX,avgY,self.depth)


		if self.gpsReceived == True:
					self.gpsReceived = False
		self.previous_time = time

		
	def max(self,liste):
		#print "----------------------"
		highest = liste[0]
		index = 0
		isUnique = True
		#print liste[0]
		for i in range(len(liste)-1):
			if liste[i+1] > highest:
				isUnique = True
				index = i+1
				highest = liste[i+1]
				nbGoodParticles = 1
			elif liste[i+1] == highest:
				isUnique = False
			#print liste[i+1]
		#print nbGoodParticles, index, liste[index]
		#print liste
		return (isUnique, index)


	def publish(self,x,y,z):
		odm = Odometry()
		rostime = rospy.get_time()
		odm.header.stamp.secs = int(rostime)
		odm.header.stamp.nsecs = 1000000000*(rostime-int(rostime))

		odm.header.frame_id = "world"

		odm.pose.pose.position.x = x
		odm.pose.pose.position.y = y
		odm.pose.pose.position.z = z

		self.pubPose.publish(odm)


	def depth_sub(self,msg):
		"""time = rospy.get_time()
		depth_measurement = msg.pose.pose.position.z
		# Kalman filtering to reduce noise
		predicted_depth = self.depth + self.dvl_z * (time-self.previous_depth_time)
		predicted_depth_cov = self.depth_cov + 0.05
		k = predicted_depth_cov/(predicted_depth_cov+0.1)
		self.depth = predicted_depth+k*(depth_measurement-predicted_depth)
		self.depth_cov = (1-k)*predicted_depth_cov
		self.previous_depth_time = time
		"""
		self.depth = msg.pose.pose.position.z

	def dvl_sub(self,msg):
		self.dvl_x = msg.velocity.x
		self.dvl_y = msg.velocity.y
		self.dvl_z = msg.velocity.z
		self.dvlReceived = True

	def imu_sub(self,msg):
		if self.dvlReceived == True:
			self.dvlReceived = False
			quaternionX = msg.orientation.x
			quaternionY = msg.orientation.y
			quaternionZ = msg.orientation.z
			quaternionW = msg.orientation.w

			# Converts the quaternion to euler
			X,Y,Z = self.quaternion_to_euler(quaternionX,quaternionY,quaternionZ,quaternionW)
			self.theta = Z

			self.filter()

	def ro_sub(self,msg):
		self.asv_x = msg.x
		self.asv_y = msg.y
		self.asv_z = msg.z
		self.range = math.sqrt(math.pow(msg.range,2) - math.pow(self.depth-self.asv_z,2))
		self.gpsReceived = True
		#print self.range

	# Converts the quaternion to euler
	def quaternion_to_euler(self,x,y,z,w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		X = math.degrees(math.atan2(t0, t1))

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.degrees(math.asin(t2))

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		Z = math.atan2(t3, t4)

		return X, Y, Z


def main(args):
	rospy.init_node('pf_estimation', anonymous=True)
	rospy.loginfo("Starting particle_filter.py")
	estim = ekf()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)