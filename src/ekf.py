#! /usr/bin/env python

"""
EKF Localisation for the AUV using dead reckoning + range-only localisation.

Input:	/desistek_saga/dvl
		/desistek_saga/imu
		/lbl_range	# 2Hz

Output: /pose/ekf

"""

import rospy
import math
import numpy as np
import sys
from nav_msgs.msg import Odometry
from go_thesis.msg import RangeOnly
from go_thesis.msg import EkfMsg
from sensor_msgs.msg import Imu
from uuv_sensor_ros_plugins_msgs.msg import DVL

class ekf:
	def __init__(self):
		# Velocity estimation of the AUV from the DVL and IMU
		self.dvl_x = 0
		self.dvl_y = 0
		self.dvl_z = 0
		self.theta = 0
		self.depth = 0
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
		self.corrected_state = np.array([[0.0],
			[0.0],
			[-80.0],
			[0.0]])
		self.matrix_A = np.array([[1.0,0.0,0.0,0.0],
			[0.0,1.0,0.0,0.0],
			[0.0,0.0,1.0,0.0],
			[0.0,0.0,0.0,1.0]])

		dvl_uncertainty = 0.05
		self.matrix_R = np.array([[dvl_uncertainty,0.0,0.0,0.0],
			[0.0,dvl_uncertainty,0.0,0.0],
			[0.0,0.0,dvl_uncertainty,0.0],
			[0.0,0.0,0.0,0.03]])

		self.matrix_Q = np.array([[1,0.0,0.0],
			[0.0,1,0.0],
			[0.0,0.0,0.05]])

		self.matrix_H = np.array([[1.0,0.0,0.0,0.0],
			[0.0,1.0,0.0,0.0],
			[0.0,0.0,1.0,0.0]])
		self.matrix_I = np.array([[1.0,0.0,0.0,0.0],
			[0.0,1.0,0.0,0.0],
			[0.0,0.0,1.0,0.0],
			[0.0,0.0,0.0,1.0]])
		self.corrected_covariance = np.array([[0.1,0.0,0.0,0.0],
			[0.0,0.1,0.0,0.0],
			[0.0,0.0,0.1,0.0],
			[0.0,0.0,0.0,0.1]])
		self.innovation = np.array([[0.],[0.],[0.]])
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
		sub_depth = rospy.Subscriber('/desistek_saga/Depth',Odometry,self.depth_sub)
		sub_ro = rospy.Subscriber('/lbl_range',RangeOnly,self.ro_sub)
		self.pubPose = rospy.Publisher('/ekf/pose',Odometry,queue_size = 1)
		self.pubEkf = rospy.Publisher('/ekf/info',EkfMsg,queue_size = 1)

	def depth_sub(self,msg):
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
		self.range = msg.range
		self.asv_x = msg.x
		self.asv_y = msg.y
		self.asv_z = msg.z
		self.gpsReceived = True

	def filter(self):
		time = rospy.get_time()
		delta_time = time-self.previous_time
		# Starts at the 2nd value
		if self.previous_time != 0:
			############ Update matrices ############
			# B
			matrix_B = np.array([[delta_time*math.cos(self.corrected_state[3]),delta_time*math.sin(self.corrected_state[3]),0.0,0.0],
				[delta_time*math.sin(self.corrected_state[3]),-delta_time*math.cos(self.corrected_state[3]),0.0,0.0],
				[0.0,0.0,delta_time,0.0],
				[0.0,0.0,0.0,delta_time]])
			# u 
			matrix_u = np.array([[self.dvl_x],
				[self.dvl_y],
				[self.dvl_z],
				[(self.theta-self.corrected_state[3])/delta_time]])
			# G
			matrix_G = np.array([[1.0,0.0,0.0,delta_time*(-matrix_u[0])*math.sin(self.corrected_state[3])+matrix_u[1]*math.cos(self.corrected_state[3])],
				[0.0,1.0,0.0,delta_time*matrix_u[0]*math.cos(self.corrected_state[3])+matrix_u[1]*math.sin(self.corrected_state[3])],
				[0.0,0.0,1.0,0.0],
				[0.0,0.0,0.0,1.0]])
			# K
			matrix_K = np.array([[0.0,0.0,0.0],
				[0.0,0.0,0.0],
				[0.0,0.0,0.0],
				[0.0,0.0,0.0]])
			# z
			matrix_z = np.array([[self.asv_x],
				[self.asv_y],
				[self.asv_z]])

			# Angles:
			varphi = math.atan2(matrix_z[1]-self.corrected_state[1],matrix_z[0]-self.corrected_state[0])
			phi = math.atan2(matrix_z[2]-self.corrected_state[2],math.sqrt(math.pow(matrix_z[0]-self.corrected_state[0],2)+math.pow(matrix_z[1]-self.corrected_state[1],2)))

			############ Observation Covariance ############
			dvl_uncertainty = 0.05*delta_time
			self.matrix_R = np.array([[dvl_uncertainty,0.0,0.0,0.0],
				[0.0,dvl_uncertainty,0.0,0.0],
				[0.0,0.0,dvl_uncertainty,0.0],
				[0.0,0.0,0.0,0.03]])
			############ Measurement Covariance ############
			# Eigen Values:
			eVal1 = 1.
			eVal2 = 0.001
			# Compute Eigen Vectors
			rot = np.array([[math.cos(varphi), -math.sin(varphi)],
				[math.sin(varphi), math.cos(varphi)]])
			
			val1 = np.array([[eVal1],
				[0]])
			eVec1 = np.dot(rot,val1)
			val2 = np.array([[0],
				[eVal2]])
			eVec2 = np.dot(rot,val2)
			# Compute Covariance Matrix
			cov = eVal1*np.dot(eVec1,eVec1.T)/(np.dot(eVec1.T,eVec1)) + eVal2*np.dot(eVec2,eVec2.T)/(np.dot(eVec2.T,eVec2))

			self.matrix_Q = np.array([[cov[0][0],cov[0][1],0.],
				[cov[1][0],cov[1][1], 0.],
				[0.,0.,0.05]])

			############ Prediction Step ############
			predicted_state =  np.dot(self.matrix_A,self.corrected_state)+np.dot(matrix_B,matrix_u)
			predicted_covariance = np.dot(matrix_G,np.dot(self.corrected_covariance,matrix_G.T)) + self.matrix_R

			# C.x
			matrix_C_dot_matrix_x = np.array([[self.corrected_state[0][0] + self.range*math.cos(varphi)*math.cos(phi)],
				[self.corrected_state[1][0] + self.range*math.sin(varphi)*math.cos(phi)],
				[self.corrected_state[2][0] + self.range*math.sin(phi)]])


			if self.gpsReceived == True:
				self.gpsReceived = False
				matrix_K = np.dot(np.dot(predicted_covariance,self.matrix_H.T),np.linalg.inv((np.dot(self.matrix_H,np.dot(predicted_covariance,self.matrix_H.T)))+self.matrix_Q))

			############ Correction Step ############
			self.innovation = matrix_z - matrix_C_dot_matrix_x
			self.corrected_state = predicted_state + np.dot(matrix_K,self.innovation)
			self.corrected_covariance = np.dot((self.matrix_I - np.dot(matrix_K,self.matrix_H)),predicted_covariance)

		self.previous_time = time
		
		self.publish(self.corrected_state[0],self.corrected_state[1],self.corrected_state[2])

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

		ek = EkfMsg()
		ek.covariance = [self.corrected_covariance[0][0],self.corrected_covariance[1][0],self.corrected_covariance[2][0],self.corrected_covariance[3][0],self.corrected_covariance[0][1],self.corrected_covariance[1][1],self.corrected_covariance[2][1],self.corrected_covariance[3][1],self.corrected_covariance[0][2],self.corrected_covariance[1][2],self.corrected_covariance[2][2],self.corrected_covariance[3][2],self.corrected_covariance[0][3],self.corrected_covariance[1][3],self.corrected_covariance[2][3],self.corrected_covariance[3][3]]
		ek.innovation = [self.innovation[0][0],self.innovation[0][0],self.innovation[0][0]]
		self.pubEkf.publish(ek)


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
	rospy.init_node('ekf_estimation', anonymous=True)
	rospy.loginfo("Starting ekf.py")
	estim = ekf()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)