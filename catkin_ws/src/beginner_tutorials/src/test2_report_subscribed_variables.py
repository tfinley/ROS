#!/usr/bin/env python

from beginner_tutorials.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import rospy

class Report():
	def __init__(self):
		self.start_time = rospy.get_rostime()
		rospy.Subscriber("chatter", String, self.chatter_callback)
		rospy.Subscriber("clicked_point", PointStamped, self.point_callback)
		self.s = rospy.Service('record_inputs', Button, self.button_callback)
		print "Ready to record."
		rospy.spin()

	def button_callback(self, req):
		print "Recording"
		print "Recording Clicked Point- x: %i, y: %i, z: %i"%(self.picked_point_x, self.picked_point_y, self.picked_point_z) 
		print "Recording time: %s"%(self.chat)
		return "Recorded"
	
	def chatter_callback(self, data):
		self.chat = data.data
		print 'chat callback'

	def point_callback(self, data):
		self.picked_point_x = data.point.x
		self.picked_point_y = data.point.y
		self.picked_point_z = data.point.z
		self.picked_point_time = data.header.stamp
		print "Time to pick: %s"%(self.picked_point_time.secs - self.start_time.secs)
		print 'point clicked callback'

def main():
	rospy.init_node('reporting_with_server')
	r1 = Report()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
