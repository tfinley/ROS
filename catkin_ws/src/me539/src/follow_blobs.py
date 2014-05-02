#!/usr/bin/env python

# Author: Taylor Finley
# Date: 2014.02.05
# Description: This node is designed to conrtrol a robot with a laserscan
#				output which this will subsribe to and determine when the 
#				robot should stop before it hits a wall. The control will
# 				be by publishing a twist msg that the robot is subscribed
# 				add param depth_registration:=false to end of launch command

# Every python controller needs these lines
import roslib; roslib.load_manifest('me539')
import rospy
import numpy
import math

# The velocity command message
from geometry_msgs.msg import Twist

# The velocity command message
from geometry_msgs.msg import Point

# Float32 message lib
#from std_msgs.msg import Float32

# Define global variable
laserOffset = 0.12
targetDist = rospy.get_param('/move/target_distance', 1.0)
targetDist = targetDist + laserOffset 
slowZone = 0.2 + targetDist
#blobArea = 0

#def size_callback(area):
#	global blobArea
#	blobArea = area.data

def blob_callback(pt):
	
	command = Twist()
	# Set other Twist data
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0

	# Forward motion decision
	blobArea = pt.z
	print ('blob_area',  blobArea)
	if blobArea < 20000:
		command.linear.x = 0.1
	else:
		command.linear.x = 0.0 # stop

	pixelDist = pt.x
	# Angle motion decision
	if pixelDist > 50:
		command.angular.z = -0.15
	elif pixelDist < -50:
		command.angular.z = 0.15
	else:
		command.angular.z = 0.0

	print('turn',  command.angular.z)

	# Publish command (Twist msg)
	pub.publish(command)



if __name__ == '__main__':
	rospy.init_node('follow_blobs')

	# A subscriber to recieve data from the blob_tracker
	sub1 = rospy.Subscriber('blobPoint', Point, blob_callback)

	# A subscriber to recieve data from the blob_tracker
	#sub2 = rospy.Subscriber('blobSize', Float32, size_callback)

	# A publisher for the move data
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

	# Do ROS stuff
	rospy.spin()

   
