#! /usr/bin/env python

"""
Get the GPS data from the heron, and convert them to XY coordinates.

Input:  /navsat/fix

Output: /heron/gps

"""
import rospy
import sys
import math
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class GPStoXY:
	def __init__(self):
		self.lat0 = 0;
		self.lon0 = 0;

		self.mdeglat = 0
		self.mdeglon = 0

		self.seq = 0;
		self.latitude = 0;
		self.longitude = 0;
		self.altitude = 0;

		self.radius = 6356752.3
		self.x0 = 0;
		self.y0 = 0;

		sub_gps = rospy.Subscriber('/navsat/fix',NavSatFix,self.gps_sub)
		self.pubXY = rospy.Publisher("/heron/gps",Odometry,queue_size=1)

	def gps_sub(self,msg):
		self.header = msg.header.seq
		self.latitude = msg.latitude;
		self.longitude = msg.longitude;
		self.altitude = msg.altitude;
		
		if self.lat0 != 0:
			self.publish_XY()
		else:
			self.lat0 = self.latitude
			self.lon0 = self.longitude
			lat0rad = math.radians(self.lat0)
			self.mdeglat = 111415.13*math.cos(lat0rad)-94.55*math.cos(3.0*lat0rad)-0.12*math.cos(5.0*lat0rad)
			self.mdeglon = 111132.09-566.05*math.cos(2.0*lat0rad)+1.20*math.cos(4.0*lat0rad)-0.002*math.cos(6.0*lat0rad)

			self.x0 = self.radius*math.sin(math.radians(90-self.latitude))*math.cos(math.radians(self.longitude))
			self.y0 = self.radius*math.sin(math.radians(90-self.latitude))*math.sin(math.radians(self.longitude))


	def publish_XY(self):
		#x = (self.longitude-self.lon0) * self.mdeglon
		#y = (self.latitude-self.lat0) * self.mdeglat
		x1 = self.radius*math.sin(math.radians(90-self.latitude))*math.cos(math.radians(self.longitude))
		y1 = self.radius*math.sin(math.radians(90-self.latitude))*math.sin(math.radians(self.longitude))
		x = x1-self.x0
		y = y1-self.y0

		rostime = rospy.get_time()
		
		odm = Odometry()
		
		odm.header.seq = self.seq
		odm.header.stamp.secs = int(rostime)
		odm.header.stamp.nsecs = 1000000000*(rostime-int(rostime))
		odm.header.frame_id = "world"

		odm.pose.pose.position.x = x
		odm.pose.pose.position.y = y
		odm.pose.pose.position.z = self.altitude

		self.pubXY.publish(odm)

def main(args):
	rospy.init_node('heron_gps',anonymous=True)
	rospy.loginfo("Starting heron_gps.py")
	convert = GPStoXY()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)