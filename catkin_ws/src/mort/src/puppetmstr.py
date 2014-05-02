#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('mort')
import rospy
import numpy
import os

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16


def master():
	global glbpkt1in,glbpkt1out,coff
	rospy.sleep(.7)
	#MAIN USER INTERFACE
	while not rospy.is_shutdown():
		os.system('clear')
		print('Manually Override Queues or Send Interrupt')
		print('  -Send interrupt request (keep sending to increase urgency): 1')
		print('  -Enter new GPS Coordinates: 2')
		
	    
		inn = int(raw_input("enter condition"))
		##Break
		if inn==1:
			coffeeget=1
			coffgt.publish(data=coffeeget)
		else:
			print('bad key')


if __name__ == '__main__':
	rospy.init_node('puppetmaster')
	coffgt = rospy.Publisher('coffee',Int16)
	master()


	rospy.spin()
