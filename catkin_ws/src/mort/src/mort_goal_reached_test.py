#!/usr/bin/env python
import roslib; roslib.load_manifest('mort')
import rospy
import tf
from  move_base_msgs.msg import MoveBaseActionResult	

def goalReachedCallback(data):

	if data.status.status == 3:
		rospy.loginfo('goal reached - status 3')
	else:
		rospy.loginfo('not status 3')

def listener():
	rospy.init_node('mort_goal_reached_test', anonymous=True)
	rospy.Subscriber("move_base/result", MoveBaseActionResult, goalReachedCallback)
	rospy.spin()	

if __name__ == '__main__':
	listener()
