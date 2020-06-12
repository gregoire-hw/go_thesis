#! /usr/bin/env python

"""
Uses the heron_controller service to reach goals
"""

import rospy
import sys
import math
from go_thesis.srv import HeronMotion, HeronMotionRequest
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

class GoToGoal:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.yaw = 0

		self.goal = 1

		sub_gps = rospy.Subscriber('/pose_gt', Odometry, self.gps_sub)
		sub_imu = rospy.Subscriber('/imu/rpy', Vector3Stamped, self.imu_sub)

	def gps_sub(self,msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

		self.command()

	def imu_sub(self,msg):
		self.yaw = msg.vector.z

	def command(self):
		rospy.wait_for_service('heron_controller')
		try:
			heron_controller = rospy.ServiceProxy('heron_controller',HeronMotion)
			cmd = heron_controller(1,0)
		
			if self.goal == 1:
				xy = [70,0]
			elif self.goal == 2:
				xy = [50,50]
			elif self.goal == 2:
				xy = [30,20]
			elif self.goal == 3:
				xy = [0,0]
			print self.goal

			theta = math.atan2(xy[1]-self.y, xy[0]-self.x)
			
			if abs(theta-self.yaw) > 0.15:
				if theta >= 0:
					if theta < self.yaw and theta >= self.yaw - math.pi:
						orientation = -1
					else:
						orientation = +1
				else:
					if theta > self.yaw and theta <= self.yaw + math.pi:
						orientation = +1
					else:
						orientation = -1

				if orientation > 0:
					cmd = heron_controller(3,0)

				else:
					cmd = heron_controller(2,0)
			else:
				cmd = heron_controller(1,0)
				
			if abs(self.x-xy[0]) < 0.5 or abs(self.y-xy[1]) < 0.5:
				self.goal += 1


			return cmd
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)


def main(args):
	rospy.init_node('heron_gotogoal',anonymous=True)
	rospy.loginfo("Starting heron_gotogoal.py")
	Start = GoToGoal()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)