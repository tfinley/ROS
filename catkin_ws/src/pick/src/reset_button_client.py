#!/usr/bin/env python
import roslib; roslib.load_manifest('pick')

import sys

import rospy
from pick.srv import *

def press_button(text):
	rospy.wait_for_service('reset_button')
	try:
		record_imputs = rospy.ServiceProxy('reset_button', Reset_Button)
		resp1 = record_imputs(text)
		print "recieved back: %s"%(resp1.response)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	press_button(sys.argv[1])
