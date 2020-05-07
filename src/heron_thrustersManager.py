#!/usr/bin/env python

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from go_thesis.msg import HeronThrusters
import rospy


def callback(cmd):
	global thruster_left
	global thruster_right

	send = FloatStamped()
	send.header.stamp = rospy.Time.now()

	send.data = cmd.left
	thruster_left.publish(send)

	send.data = cmd.right
	thruster_right.publish(send)

def init():
	global thruster_left
	global thruster_right

	namespace = "heron"

	thruster_left = rospy.Publisher("/"+namespace+"/thrusters/1/input",FloatStamped,queue_size=1)
	thruster_right = rospy.Publisher("/"+namespace+"/thrusters/0/input",FloatStamped,queue_size=1)

	rospy.Subscriber("/heron_controls",HeronThrusters,callback)

	rospy.spin()

if __name__ == '__main__':
	rospy.init_node("heron_thrusters_manager")
	init()