#!/usr/bin/env python

from pick.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from random import randint
import rospy
import rospkg

class Report():
	def __init__(self):
		#Mark Start Time
		self.start_time = rospy.get_rostime()

		#Get path to package
		rospack = rospkg.RosPack()
		self.package_path = rospack.get_path('pick')

		# Marker object
		self.marker = Marker()

		#Subscriber, publish, server
		rospy.Subscriber("clicked_point", PointStamped, self.point_callback)
		self.s1 = rospy.Service('confirm_button', Confirm_Button, self.confirm_button_callback)
		self.s2 = rospy.Service('reset_button', Reset_Button, self.reset_button_callback)
		self.publisher = rospy.Publisher("picked_point_marker", Marker)

		#Parameters
		self.scene_number = randint(1,20)
		self.part_number = randint(1,4)
		rospy.set_param('/study/scene_number', self.scene_number)
		rospy.set_param('/study/part_number', self.part_number)
		self.iter = rospy.get_param('/study/iter') # set iter based on param - this will help if it crashes mid cycle. set using argument in launch file - 'iter:=12' (to start on iteration 12)
		self.user_number = rospy.get_param('/study/user_number')
		self.hmi = rospy.get_param('study/hmi')

		#Log info to start
		rospy.loginfo('User Number: %i', self.user_number)
		rospy.loginfo('HMI: %s', self.hmi)

		#Spin and wait
		print "Ready"
		rospy.spin()

	def confirm_button_callback(self, req):	
		#Note the time and calculate duration from start
		self.confirmation_time = rospy.get_rostime()
		self.confirmation_duration = self.confirmation_time - self.picked_point_time

		#Get parameters to record
		self.scene_number = rospy.get_param('/study/scene_number')
		self.part_number = rospy.get_param('/study/part_number')

		#Log info for record
		rospy.loginfo('Recording Iter: %i', self.iter)
		rospy.loginfo('Scene Number: %i', self.scene_number)
		rospy.loginfo('Part Number: %i', self.part_number)

		#Recording Into Text File
		self.record_path = self.package_path + '/recorded_data/' + str(self.user_number) + '/' + str(self.hmi) + '_' + str(self.iter) + '.txt'
		record_string = str(self.scene_number) + ' ' \
			+ str(self.part_number) + ' ' \
			+ str(self.picked_point_x) + ' ' \
			+ str(self.picked_point_y) + ' ' \
			+ str(self.picked_point_z) + ' ' \
			+ str(self.picked_point_duration.secs) + '.' + str(self.picked_point_duration.nsecs) + ' ' \
			+ str(self.confirmation_duration.secs) + '.' + str(self.confirmation_duration.nsecs)
		f = open(self.record_path, 'w')
		f.write(record_string)
		f.close()

		# Setup for next iteration
		self.iter = self.iter + 1
		self.start_time = rospy.get_rostime() # Restart time
		self.scene_number = randint(1,20)
		self.part_number = randint(1,4)
		rospy.set_param('/study/scene_number', self.scene_number)
		rospy.set_param('/study/part_number', self.part_number)

		return "Recorded"

	def reset_button_callback(self, req):
		print "Reset Button Pressed"
		return "Reset"

	def point_callback(self, data):
		#Get point data
		self.picked_point_x = data.point.x
		self.picked_point_y = data.point.y
		self.picked_point_z = data.point.z

		#Get time and duration for clicked point
		self.picked_point_time = data.header.stamp
		self.picked_point_duration = self.picked_point_time - self.start_time

		#Print out
		print 'point clicked callback'
		rospy.loginfo("Clicked Point- x: %f, y: %f, z: %f", self.picked_point_x, self.picked_point_y, self.picked_point_z)
		#print "Clicked Point- x: %f, y: %f, z: %f"%(self.picked_point_x, self.picked_point_y, self.picked_point_z) 



def main():
	rospy.init_node('study_master')
	r1 = Report()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
