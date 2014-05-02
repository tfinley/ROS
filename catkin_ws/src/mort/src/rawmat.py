#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('mort')
import rospy
import random


from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16



##Updates
def rawmatp(data):
	a = round(random.random()+.1)
	if a==1:
		out = [1,1]
	else:
		out = [1.01,1.01]
	print(out)
	rawmats.publish(data=out)	

if __name__ == '__main__':
	rospy.init_node('rawmat')
	rawmats = rospy.Publisher('rawmatout',Float32MultiArray)
	rospy.Subscriber('rawmatreq',Int16,rawmatp)
	rospy.spin()
