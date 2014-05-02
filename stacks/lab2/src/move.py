#!/usr/bin/env python

# Author: Taylor Finley
# Date: 2014.02.05
# Description: This node is designed to conrtrol a robot with a laserscan
#				output which this will subsribe to and determine when the 
#				robot should stop before it hits a wall. The control will
# 				be by publishing a twist msg that the robot is subscribed
# 				add param depth_registration:=false to end of launch command

# Every python controller needs these lines
import roslib; roslib.load_manifest('lab2')
import rospy
import math

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
minAngle = math.radians(-45) #set min angle to 15 degrees left of center
maxAngle = math.radians(45) #set min angle to 15 degrees right of center

def scan_callback(data):
	subsetDist = []
	# Calculate min distance from laserscan (data)
	dists = data.ranges
	# Get min angle
	minSensorAngle = data.angle_min
	print("min sensor angle", minSensorAngle)
	# Get anlge increment between range values
	angleInc = data.angle_increment
	# Calcultee min distance between desired angles
	for i in range(len(dists)):
		#calculate angles
		angle = minSensorAngle + angleInc*i
		if angle >= minAngle and angle <= maxAngle:
			subsetDist.append(dists[i])
	minSubsetDist = min(subsetDist)
	rospy.loginfo(minSubsetDist)
	dist = minSubsetDist
 
	# Drive forward at a given speed.  The robot points up the x-axis.
	command = Twist()

	# Forward motion decision
	if dist > targetDist and abs(dist-targetDist) > 0.005:
		if dist > slowZone:
			command.linear.x = 0.1
		else:
			command.linear.x = 0.05
	elif dist < targetDist and abs(dist-targetDist) > 0.005:
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
	sub = rospy.Subscriber('scan', LaserScan, scan_callback)

	# A publisher for the move data
	pub = rospy.Publisher('cmd_vel', Twist)

	# A publisher for minimum distance read
	pub2 = rospy.Publisher('dist', Float32)

	# Do ROS stuff
	rospy.spin()

   
