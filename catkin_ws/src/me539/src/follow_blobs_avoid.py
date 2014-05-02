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
import numpy as np

# The velocity command message
from geometry_msgs.msg import Twist

# The velocity command message
from geometry_msgs.msg import Point

# LaserScan message lib
from sensor_msgs.msg import LaserScan

# Define global variable
blobArea = 1.1
blobX = 1.1
xMin = -0.5
xMax = 0.5

#def size_callback(area):
#	global blobArea
#	blobArea = area.data

def blob_callback(pt):
	global blobX
	global blobArea
	blobX = pt.x
	blobArea = pt.z

def scan_callback(data):
	global blobX
	global blobArea
	pts=np.array([100,100]) # create ~empty numpy array for points
	dists = data.ranges # get distances from laserscan
	# filter out nan values
	distArray = np.array(dists) #create array from distances
	whereNans = np.isnan(distArray) # find where nan values are
	distArray[whereNans] = 100 # replace all nan values with 100
	# Get min angle
	minSensorAngle = data.angle_min
	# Get anlge increment between range values
	angleInc = data.angle_increment
	# Calcultee min distance in ROAD
	for i in range(len(dists)):
		#calculate angles
		theta = minSensorAngle + angleInc*i #remember that i starts with 0
		xtemp = distArray[i] * math.sin(theta) *-1
		ytemp = distArray[i]*math.cos(theta)
		temp = np.array([xtemp, ytemp])
		pts = np.vstack((pts,temp))
	subPts = pts[np.all([pts[:,0] > xMin, pts[:,0] < xMax], axis=0)] # filter points in road
	minIndex = np.argmin(subPts[:,1]) #get index of min y
	obstaclePtX = subPts[minIndex,0] #get x value of obstacle in the road
	obstaclePtY = subPts[minIndex,1] #get y value of obstacle in the road	
	if obstaclePtY > 1:
		roadClear = True
	else:
		roadClear = False

	# Make decision about obstacle in the road
	if obstaclePtY < 1.0:
		print 'STOP'
	else:
		print 'GO'

	#SET COMMAND
	command = Twist()
	# Set other Twist data
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0

	# Forward motion decision
	if blobArea > 10000: 	#  reached the blob goal
		command.linear.x = 0.0
		command.angular.z = 0.0
	elif blobArea < 10000 and roadClear: 	# no obstacle, keep going
		command.linear.x = 0.2
	elif blobArea < 10000 and roadClear is False and obstaclePtY > 0.5: # obstacle avoidance zone
		command.linear.x = 0.2 		# slow down around obstacle
		if obstaclePtX > 0: 				# obstacle is on the right side of the road
			command.angular.z = 0.5 	# turn left
		else: 											# obstacle is on the left side of the road
			command.angular.z = -0.5 # turn right
	else: 												# too close
		command.linear.x = -0.05		# back up
	

	# Angle motion decision if road is clear
	if roadClear:
		if blobX > 50:
			command.angular.z = -0.15
		elif blobX < -50:
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

	# A subscriber to recieve data from the robot's laserscan
	sub2 = rospy.Subscriber('scan', LaserScan, scan_callback)

	# A publisher for the move data
	pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)

	# Do ROS stuff
	rospy.spin()

   
