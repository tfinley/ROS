#!/usr/bin/env python
import roslib; roslib.load_manifest('mort')
import rospy
from visualization_msgs.msg import *


def setupWorkbenches():
	rospy.init_node('mort_workbench_markers', anonymous=True)
	marker_pub = rospy.Publisher('/mort/workbench_marker', visualization_msgs.msg.Marker)
	marker = visualization_msgs.msg.Marker()
	print "publishing markers"
	marker.header.frame_id = 'map'
	marker.pose.position.x = 2.4186
	marker.pose.position.y = -0.6013
	marker.pose.position.z = 1.0
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.header.stamp = rospy.Time.now()
	marker.ns = "workbench1"
	marker.id = 1
	marker.action = marker.ADD
	marker.type = marker.SPHERE
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.scale.x = 0.5
	marker.scale.y = 0.5
	marker.scale.z = 0.5
	print marker
	marker_pub.publish(marker);


	# Spin
	rospy.spin()	

if __name__ == '__main__':
	setupWorkbenches()
