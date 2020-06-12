#! /usr/bin/env python


import rospy
import math
import numpy as np
import sys
from nav_msgs.msg import Odometry
from go_thesis.msg import EkfMsg
import csv
import os
import termios, fcntl


class Save:
	def __init__(self):
		self.dvl_time = []
		self.dvl_x = [] 
		self.dvl_y = []
		self.dvl_z = []
		self.gt_time = []
		self.gt_x = []
		self.gt_y = []
		self.gt_z = []
		self.ekf_time = []
		self.ekf_x = []
		self.ekf_y = []
		self.ekf_z = []
		self.ekf_x_f = []
		self.ekf_y_f = []
		self.ekf_z_f = []
		self.cov = []
		self.innov = []
			
		sub_dvl = rospy.Subscriber('/desistek_saga/dvl/position',Odometry,self.dvl_sub)
		sub_desistek = rospy.Subscriber('/desistek_saga/pose_gt',Odometry,self.desistek_sub)
		sub_ekf_pose = rospy.Subscriber('/ekf/pose',Odometry,self.ekf_pose_sub)
		sub_ekf_info = rospy.Subscriber('/ekf/info',EkfMsg,self.ekf_info_sub)
		sub_ekf_filtered = rospy.Subscriber('/ekf/pose/filtered',Odometry,self.ekf_filtered_sub)

		self.saveData()

	def dvl_sub(self,msg):
		self.dvl_time.append(rospy.get_time())
		self.dvl_x.append(msg.pose.pose.position.x)
		self.dvl_y.append(msg.pose.pose.position.y)
		self.dvl_z.append(msg.pose.pose.position.z)

	def desistek_sub(self,msg):
		self.gt_time.append(rospy.get_time())
		self.gt_x.append(msg.pose.pose.position.x)
		self.gt_y.append(msg.pose.pose.position.y)
		self.gt_z.append(msg.pose.pose.position.z)

	def ekf_pose_sub(self,msg):
		self.ekf_time.append(rospy.get_time())
		self.ekf_x.append(msg.pose.pose.position.x)
		self.ekf_y.append(msg.pose.pose.position.y)
		self.ekf_z.append(msg.pose.pose.position.z)

	def ekf_filtered_sub(self,msg):
		self.ekf_x_f.append(msg.pose.pose.position.x)
		self.ekf_y_f.append(msg.pose.pose.position.y)
		self.ekf_z_f.append(msg.pose.pose.position.z)

	def ekf_info_sub(self,msg):
		self.cov.append(msg.covariance)
		self.innov.append(msg.innovation)

	def saveData(self):
		try:
			c = sys.stdin.read(1)
			print c
			if c == 'p' or c == 'P':
				rospy.loginfo("Saving Data")
				path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "data/DR.csv")
				with open(path, mode='w') as csv_file:
					writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
					writer.writerow(['time','X','Y','Z'])
					for i in range(len(self.dvl_x)):
						writer.writerow([self.dvl_time[i],self.dvl_x[i],self.dvl_y[i],self.dvl_z[i]])
					
				path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "data/gt.csv")
				with open(path, mode='w') as csv_file:
					writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
					writer.writerow(['time','X','Y','Z'])
					for i in range(len(self.gt_x)):
						writer.writerow([self.gt_time[i],self.gt_x[i],self.gt_y[i],self.gt_z[i]])

				path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "data/ekf.csv")
				with open(path, mode='w') as csv_file:
					writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
					writer.writerow(['time','X','Y','Z','CovXX','CovXY','CovXZ','CovXYaw','CovYX','CovYY','CovYZ','CovYYaw','CovZX','CovZY','CovZZ','CovZYaw','CovYawX','CovYawY','CovYawZ','CovYawYaw','InnovX','InnovY','InnovZ'])
					for i in range(len(self.ekf_x)):
						writer.writerow([self.ekf_time[i],self.ekf_x[i],self.ekf_y[i],self.ekf_z[i],self.cov[i][0],self.cov[i][1],self.cov[i][2],self.cov[i][3],self.cov[i][4],self.cov[i][5],self.cov[i][6],self.cov[i][7],self.cov[i][8],self.cov[i][9],self.cov[i][10],self.cov[i][11],self.cov[i][12],self.cov[i][13],self.cov[i][14],self.cov[i][15],self.innov[i][0],self.innov[i][1],self.innov[i][2]])
				path = os.path.join(os.path.abspath(os.path.dirname(__file__)), "data/ekf_filtered.csv")
				with open(path, mode='w') as csv_file:
					writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
					writer.writerow(['time','X','Y','Z'])
					for i in range(len(self.ekf_x)):
						writer.writerow([self.ekf_time[i],self.ekf_x_f[i],self.ekf_y_f[i],self.ekf_z_f[i]])
				rospy.loginfo("Save completed")
		except IOError: pass

def main(args):
	rospy.init_node('saveEKF', anonymous=True)
	rospy.loginfo("Starting save_data.py")
	start = Save()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		
if __name__ == '__main__':
	main(sys.argv)