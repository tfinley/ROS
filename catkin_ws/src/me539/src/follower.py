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

# Define global variable
laserOffset = 0.12
targetDist = rospy.get_param('/move/target_distance', 1.0)
targetDist = targetDist + laserOffset 
slowZone = 0.2 + targetDist

def point_callback(pt):
	#calculate dist and angle
	dist = math.sqrt((pt.x**2)+(pt.y**2))
	theta = math.atan(pt.x/pt.y)
	command = Twist()
	# Set other Twist data
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0

	# Forward motion decision
	dist = pt.y
	if dist > targetDist and abs(dist-targetDist) > 0.010:
		if dist > slowZone:
			command.linear.x = 0.4
		else:
			command.linear.x = 0.1
	elif dist < targetDist and abs(dist-targetDist) > 0.010:
		command.linear.x = -0.05 # move back slowly
	else:
		command.linear.x = 0.0
		print("stop")

	# Angle motion decision
	if theta > 0.1:
		command.angular.z = -0.75
	elif theta < -0.1:
		command.angular.z = 0.75
	else:
		command.angular.z = 0.0

	print('forward', command.linear.x,'turn',  command.angular.z)

	# Publish command (Twist msg)
	pub.publish(command)



if __name__ == '__main__':
	rospy.init_node('follower')

	# A subscriber to recieve data from the robot's laserscan
	sub = rospy.Subscriber('followPoint', Point, point_callback)

	# A publisher for the move data
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

	# Do ROS stuff
	rospy.spin()

   
