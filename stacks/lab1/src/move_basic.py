#!/usr/bin/env python

# Author: Taylor Finley
# Date: 2014.01.22
# Description: This node is designed to conrtrol a robot with a laserscan
#				output which this will subsribe to and determine when the 
#				robot should stop before it hits a wall. The control will
# 				be by publishing a twist msg that the robot is subscribed

# Every python controller needs these lines
import roslib; roslib.load_manifest('lab1')
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# LaserScan message lib
from sensor_msgs.msg import LaserScan

# Float32 message lib
from std_msgs.msg import Float32

# Define global variable
laserOffset = 0.12
targetDist = rospy.get_param('/move/target_distance', .5)
targetDist = targetDist + laserOffset 
slowZone = 0.2 + targetDist

def scan_callback(data):

	# Calculate min distance from laserscan (data)
	dist = min(data.ranges)
	rospy.loginfo(dist)
 
	# Drive forward at a given speed.  The robot points up the x-axis.
	command = Twist()

	# Forward motion decision
	if dist > targetDist and abs(dist-targetDist) > 0.015:
		if dist > slowZone:
			command.linear.x = 0.1
		else:
			command.linear.x = 0.025
	elif dist < targetDist and abs(dist-targetDist) > 0.015:
		command.linear.x = -0.01 # move back slowly
	else:
		command.linear.x = 0.0

	# Set other Twist data
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	command.angular.z = 0.0

	# Publish command (Twist msg)
	pub.publish(command)

	# Pubish distance out
	pub2.publish(dist-laserOffset)



if __name__ == '__main__':
	rospy.init_node('move')

	# A subscriber to recieve data from the robot's laserscan
	sub = rospy.Subscriber('base_scan', LaserScan, scan_callback)

	# A publisher for the move data
	pub = rospy.Publisher('cmd_vel', Twist)

	# A publisher for minimum distance read
	pub2 = rospy.Publisher('dist', Float32)

	# Do ROS stuff
	rospy.spin()

   
