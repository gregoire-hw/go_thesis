#!/usr/bin/env python

"""
Data sent by this program are computed by /heron_simulator/heron_gazebo/script/cmd_drive_translate.py
"""

from go_thesis.msg import HeronThrusters
import rospy
import sys

class Controller:
	def __init__(self):
		self.pubControl = rospy.Publisher("/heron_controls",HeronThrusters,queue_size=1)
		self.left = 1.0
		self.right = 1.0

		self.sendToThrusters()

	# /!\ This function must be in a loop to work!
	def sendToThrusters(self):
		drive = HeronThrusters()
		drive.left = self.left
		drive.right = self.right
		self.pubControl.publish(drive)

def main(args):
	rospy.init_node('heron_control',anonymous=True)
	controller = Controller()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	rospy.loginfo("Starting heron_controller.py")
	main(sys.argv)