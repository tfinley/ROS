#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

class Report():
	def __init__(self):
		self.s = rospy.Service('record_inputs', Button, self.button_callback)
		print "Ready to record."
		rospy.spin()

	def button_callback(self, req):
		print "Recording"
		return "Recorded"

def main():
	rospy.init_node('reporting_with_server')
	r1 = Report()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
