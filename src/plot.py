#! /usr/bin/env python


import numpy as numpy
from matplotlib import pyplot as plt
import math
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from go_thesis.msg import RangeOnly
from std_msgs.msg import Float32

def particles_sub(msg):
	global counter
	global asv_x
	global asv_y
	global rov_x
	global rov_y
	global pf_x
	global pf_y
	global pubErr

	x = []
	y = []
	if counter % 5 == 0:
		for i in range(len(msg.points)):
			x.append(msg.points[i].x)
			y.append(msg.points[i].y)
		plt.clf()
		plt.scatter(x, y, c="b", marker="+")
		plt.scatter([asv_x],[asv_y], c="g")
		plt.scatter([rov_x],[rov_y], c="r")
		plt.scatter([pf_x],[pf_y], c="black")
		plt.axis("equal")
		plt.draw()
		plt.pause(0.00000000005)


		err = Float32()
		err.data = math.sqrt(math.pow(rov_x-pf_x,2)+math.pow(rov_y-pf_y,2))
		pubErr.publish(err)

	counter += 1

def ro_sub(msg):
	global asv_x; asv_x = msg.x
	global asv_y; asv_y = msg.y
	asv_x = 0
	asv_y = 0

def rov_sub(msg):
	global rov_x; rov_x = msg.pose.pose.position.x
	global rov_y; rov_y = msg.pose.pose.position.y

def pf_pose_sub(msg):
	global pf_x; pf_x = msg.pose.pose.position.x
	global pf_y; pf_y = msg.pose.pose.position.y

if __name__ == '__main__':
	counter = 0

	rospy.init_node("plotter")
	rospy.Subscriber('/particle_filter/particles', PointCloud, particles_sub)
	rospy.Subscriber('/lbl_range',RangeOnly, ro_sub)
	rospy.Subscriber('/desistek_saga/pose_gt', Odometry, rov_sub)
	rospy.Subscriber('/particle_filter/pose',Odometry, pf_pose_sub)
	global pubErr
	pubErr = rospy.Publisher('/particle_filter/error',Float32, queue_size=1)

	plt.ion()
	plt.show()
	rospy.spin()