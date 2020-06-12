#!/usr/bin/env python

"""
Trilateration localisation using NLS method.

Input:	/desistek_saga/dvl
		/desistek_saga/imu
		/pose_gt
		/lbl_range

Output:	/pose/trilateration
"""

import rospy
import math
import numpy as np
import sys
from nav_msgs.msg import Odometry
from go_thesis.msg import RangeOnly
from sensor_msgs.msg import Imu
from uuv_sensor_ros_plugins_msgs.msg import DVL

class Trilateration:
	def __init__(self):
		self.nbBeacons = 5
		self.start = False
		self.timeDVL = rospy.get_time()
		self.previous_time = self.timeDVL
		self.gps_rate = 3
		self.gps_counter = 0
		# Velocity estimation of the AUV from the DVL and IMU
		self.dvl_x = 0.
		self.dvl_y = 0.
		self.dvl_z = 0.
		self.theta = 0.
		self.add_x = [0.,0.,0.,0.,0.]
		self.add_y = [0.,0.,0.,0.,0.]
		self.add_z = [0.,0.,0.,0.,0.]
		# pose estimation of the ASV from the GPS
		self.asv_x = [0.,0.,0.,0.,0.]
		self.asv_y = [0.,0.,0.,0.,0.]
		self.asv_z = [0.,0.,0.,0.,0.]
		# Range between the 2 robots
		self.range = [0.,0.,0.,0.,0.]
		# Position obtained after NLS
		self.pose_estimate_x = 0.
		self.pose_estimate_y = 0.
		self.pose_estmiate_z = 0.
		# Other needed variables
		self.previous_time = 0
		self.previous_theta = 0
		self.gpsReceived = False
		self.dvlReceived = False
		self.counter = 0
		self.beacon = 0
		# Matrices
		self.matrix_f = np.array([[0.],
			[0.],
			[0.],
			[0.],
			[0.]])
		self.matrix_J = np.array([[0.,0.,0.],
			[0.,0.,0.],
			[0.,0.,0.],
			[0.,0.,0.],
			[0.,0.,0.]])
		self.matrix_R = np.array([[4.],
			[4.],
			[-80.]])
		# Topics
		sub_dvl = rospy.Subscriber('/desistek_saga/dvl',DVL,self.dvl_sub)
		sub_imu = rospy.Subscriber('/desistek_saga/imu', Imu, self.imu_sub) 
		sub_ro = rospy.Subscriber('/lbl_range',RangeOnly,self.ro_sub)
		self.pubPose = rospy.Publisher('/pose/trilateration',Odometry,queue_size = 1)

	def trilateration(self):
		self.add_x[self.beacon] = 0
		self.add_y[self.beacon] = 0
		self.add_z[self.beacon] = 0

		self.beacon += 1
		if self.beacon >= self.nbBeacons:
			self.beacon = 0
			self.start = True

		self.nls()

	def nls(self):
		if self.start == True:
			# Do the iteration 10 times
			#print self.asv_x
			#print self.asv_y
			#print self.asv_z
			#print self.range
			#print "---------"
			for j in range(100):
				for i in range(self.nbBeacons):
					self.matrix_f[i][0] = math.sqrt(math.pow(self.matrix_R[0][0]-(self.asv_x[i]+self.add_x[i]),2)+math.pow(self.matrix_R[1][0]-(self.asv_y[i]+self.add_y[i]),2)+math.pow(self.matrix_R[2][0]-(self.asv_z[i]+self.add_z[i]),2))-self.range[i]
					self.matrix_J[i][0] = (self.matrix_R[0][0]-self.asv_x[i])/(self.matrix_f[i][0]+self.range[i])
					self.matrix_J[i][1] = (self.matrix_R[1][0]-self.asv_y[i])/(self.matrix_f[i][0]+self.range[i])
					self.matrix_J[i][2] = (self.matrix_R[2][0]-self.asv_z[i])/(self.matrix_f[i][0]+self.range[i])
				# Newton iteration:
				self.matrix_R = self.matrix_R - np.dot(np.linalg.inv(np.dot(self.matrix_J.T,self.matrix_J)),np.dot(self.matrix_J.T,self.matrix_f))
		self.publish()

	def publish(self):
		odm = Odometry()
		rostime = rospy.get_time()
		odm.header.stamp.secs = int(rostime)
		odm.header.stamp.nsecs = 1000000000*(rostime-int(rostime))

		odm.header.frame_id = "world"

		odm.pose.pose.position.x = self.matrix_R[0][0]
		odm.pose.pose.position.y = self.matrix_R[1][0]
		odm.pose.pose.position.z = self.matrix_R[2][0]

		self.pubPose.publish(odm)

	# Computes the distance parcoured by the AUV thanks to the DVL. Save them for each beacons
	def dvlMoves(self):
		self.timeDVL = rospy.get_time()
		dt = float(self.timeDVL - self.previous_time)
		x = dt*(self.dvl_x*math.cos(self.theta) + self.dvl_y*math.sin(self.theta))
		y = dt*(-self.dvl_x*math.sin(self.theta) + self.dvl_y*math.cos(self.theta))
		z = self.dvl_z*dt

		for i in range(self.nbBeacons):
			self.add_x[i] += x
			self.add_y[i] += y
			self.add_z[i] += z

		self.previous_time = rospy.get_time()

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

			self.dvlMoves()

	def ro_sub(self,msg):
		self.gps_counter += 1
		if self.gps_counter >= self.gps_rate:
			self.gps_counter = 0
			self.range[self.beacon] = msg.range
			self.asv_x[self.beacon] = msg.x
			self.asv_y[self.beacon] = msg.y
			self.asv_z[self.beacon] = msg.z

			self.trilateration()

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
	rospy.init_node('trilateration_estimation', anonymous=True)
	rospy.loginfo("Starting trilateration.py")
	estim = Trilateration()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)