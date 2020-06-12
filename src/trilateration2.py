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
		p1 = np.array([[0.],[0.]])
		self.p = [p1,p1,p1,p1,p1]
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
		# Topics
		sub_dvl = rospy.Subscriber('/desistek_saga/dvl',DVL,self.dvl_sub)
		sub_imu = rospy.Subscriber('/desistek_saga/imu', Imu, self.imu_sub) 
		sub_ro = rospy.Subscriber('/lbl_range',RangeOnly,self.ro_sub)
		self.pubPose = rospy.Publisher('/pose/trilateration_a',Odometry,queue_size = 1)

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
		I = np.array([[1,0],[0,1]])
		# Calculates a, B, c
		a = 0
		B = 0
		c = 0
		for i in range(self.nbBeacons):
			a += np.dot(self.p[i],np.dot(self.p[i].T,self.p[i])) - math.pow(self.range[i],2)
			B += -2*np.dot(self.p[i],self.p[i].T) - np.dot(self.p[i].T,self.p[i])*I + math.pow(self.range[i],2)*I
			c += self.p[i]
		a = a/self.nbBeacons
		B = B/self.nbBeacons
		c = c/self.nbBeacons

		# Calculates f and f'
		f = a + np.dot(B,c) + 2*np.dot(c,np.dot(c.T,c))
		fp = f[0]

		# Calculates H and H'
		H = 0
		for i in range(self.nbBeacons):
			H += np.dot(self.p[i],self.p[i].T)
		H = -2*H/self.nbBeacons + 2*np.dot(c,c.T)
		Hp = np.array([H[0][0]-H[1][0], H[0][1]-H[1][1]])

		# Calculates Q and U:
		Q = 1
		U = Hp

		# Calculates qTq:
		qTq = 0
		for i in range(self.nbBeacons):
			qTq = math.pow(self.range[i],2) - np.dot(self.p[i].T,self.p[i])
		qTq = qTq/self.nbBeacons + np.dot(c.T,c)

		#Calculates q1 and q2:
		v = Q*fp
		u1 = U[0]
		u2 = U[1]
		deltaq2 = math.pow(2*v*u2/math.pow(u1,2),2) - 4*(1+math.pow(u2/u1,2))*(math.pow(v/u1,2)-qTq)
		print deltaq2
		q2 = (-(2*v*u2/math.pow(u1,2)) - math.sqrt(abs(deltaq2)))/(2*(1+(math.pow(u2/u1,2))))
		q1 = -v/u1 - u2/u1 * q2

		x = q1 + c[0]
		y = q2 + c[1]
		z = 4

		self.publish(x,y,z)

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


	def ro_sub(self,msg):
		self.gps_counter += 1
		if self.gps_counter >= self.gps_rate:
			self.gps_counter = 0
			self.range[self.beacon] = math.sqrt(math.pow(msg.range,2)-math.pow(5.14,2))
			self.p[self.beacon] = np.array([[msg.x],[msg.y]])

			self.trilateration()

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
	rospy.init_node('trilateration_estimation2', anonymous=True)
	rospy.loginfo("Starting trilateration.py")
	estim = Trilateration()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)