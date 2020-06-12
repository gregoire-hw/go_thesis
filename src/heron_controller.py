#!/usr/bin/env python

"""
Data sent by this program are computed by /heron_simulator/heron_gazebo/script/cmd_drive_translate.py
"""

from go_thesis.msg import HeronThrusters
import rospy
import sys
from go_thesis.srv import HeronMotion, HeronMotionResponse


def start_server():
	global pubControl
	rospy.init_node('heron_control',anonymous=True)
	pubControl = rospy.Publisher("/heron_controls",HeronThrusters,queue_size=1)
	s = rospy.Service('heron_controller', HeronMotion, handle_heron_controls)
	rospy.loginfo("Starting heron_controller.py")
	rospy.spin()

def handle_heron_controls(req):
	resp = HeronMotionResponse()
	# Get Direction:
	if req.direction == 0:
		stop()
	elif req.direction == 1:
		forward()
	elif req.direction == 2:
		circle_clockwise(req.radius)
	elif req.direction == 3:
		circle_anticlockwise(req.radius)
	resp.success = True
	return resp


def stop():
	global pubControl
	drive = HeronThrusters()
	drive.left = 0.7
	drive.right = 0.7
	pubControl.publish(drive)

def forward():
	global pubControl
	drive = HeronThrusters()
	drive.left = 0.7
	drive.right = 0.7
	pubControl.publish(drive)

def circle_clockwise(radius):
	global pubControl
	drive = HeronThrusters()
	drive.left = 0.7
	drive.right = (radius-0.377654)/(radius+0.377654)
	pubControl.publish(drive)

def circle_anticlockwise(radius):
	global pubControl
	drive = HeronThrusters()
	drive.left = (radius-0.377654)/(radius+0.377654)
	drive.right = 0.7
	pubControl.publish(drive)


if __name__ == '__main__':
	start_server()