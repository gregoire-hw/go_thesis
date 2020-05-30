#! /usr/bin/env python

"""
Converts pressure from pressure_sensor into the depth of the AUV.

Input:	/desistek_saga/pressure

Output:	/desistek_saga/depth
"""

import rospy
from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry
import sys

class Pressure:
	def __init__(self):
		self.standardPressure = 101.325
		self.kPaPerM = 9.80638

		sub_pressure = rospy.Subscriber('/desistek_saga/pressure', FluidPressure, self.pressure_sub)
		self.pub = rospy.Publisher('/desistek_saga/depth', Odometry, queue_size=1)

	def pressure_sub(self,msg):
		pressure = msg.fluid_pressure
		depth = (pressure-self.standardPressure)/self.kPaPerM

		dep = Odometry()
		dep.pose.pose.position.z = -depth
		self.pub.publish(dep)

def main(args):
	rospy.init_node('depth_sensor',anonymous=True)
	rospy.loginfo("Starting desistek_depth.py")
	press = Pressure()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
