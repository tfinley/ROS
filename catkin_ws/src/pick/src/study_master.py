#!/usr/bin/env python

from pick.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from random import *
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
		self.marker.header.frame_id = 'camera_rgb_optical_frame'
		self.marker.type = Marker.SPHERE
		self.marker.id = 1
		self.ns = 'my_marker'
		self.marker.action = Marker.ADD #try delete here
		self.marker.scale.x = .01
		self.marker.scale.y = .01
		self.marker.scale.z = .00
		self.marker.pose.position.x = 0
		self.marker.pose.position.y = 0
		self.marker.pose.position.z = 0
		self.marker.color.r = 1.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0
		self.marker.color.a = 1.0

		#Subscriber, publish, server
		rospy.Subscriber("clicked_point", PointStamped, self.point_callback)
		self.s1 = rospy.Service('confirm_button', Confirm_Button, self.confirm_button_callback)
		self.s2 = rospy.Service('reset_button', Reset_Button, self.reset_button_callback)
		self.publisher = rospy.Publisher("picked_point_marker", Marker)

		#Parameters
		seed()
		self.scene_number = randint(1,20)
		self.part_number = randint(1,4)
		rospy.set_param('/study/scene_number', self.scene_number)
		rospy.set_param('/study/part_number', self.part_number)
		rospy.set_param('/study/instruction_number', 1)
		self.iter = rospy.get_param('/study/iter') # set iter based on param - this will help if it crashes mid cycle. set using argument in launch file - 'iter:=12' (to start on iter 12)
		self.user_number = rospy.get_param('/study/user_number')
		self.hmi = rospy.get_param('study/hmi')
		self.clicked_point_bool = False
		self.reset_instances = 0
		self.max_iter = 20

		#Log info to start
		rospy.loginfo('User Number: %i', self.user_number)
		rospy.loginfo('HMI: %s', self.hmi)

		#create report map text file
		self.readme_path = self.package_path + '/recorded_data/' + str(self.user_number) + '/' +'README.txt'
		f = open(self.readme_path, 'w')
		f.write('scene_number part_number picked_point_x picked_point_y picked_point_z picked_point_duration confirmation_duration reset_instances\n')
		f.write('picked_point_duration: seconds from a new scene to the time the last point was picked')
		f.write('confirmation_duration: seconds from the last point that was picked to when the confirmation button was clicked')
		f.write('reset_instances: the number of times the user cliced the reset button for this scene')
		f.close()

		#Spin and wait
		print "Ready"
		while not rospy.is_shutdown():
			self.publisher.publish(self.marker)
			rospy.sleep(.01)

	def confirm_button_callback(self, req):	
		if self.clicked_point_bool:
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
				+ str(self.confirmation_duration.secs) + '.' + str(self.confirmation_duration.nsecs) + ' ' \
				+ str(self.reset_instances)
			f = open(self.record_path, 'w')
			f.write(record_string)
			f.close()

			# check to see if it was the last iter
			if self.iter < self.max_iter:
				# Setup for next iter
				seed()
				self.iter = self.iter + 1
				self.start_time = rospy.get_rostime() # Restart time
				self.scene_number = randint(1,20)
				self.part_number = randint(1,4)
				rospy.set_param('/study/scene_number', self.scene_number)
				rospy.set_param('/study/part_number', self.part_number)
				rospy.set_param('/study/instruction_number', 1)
				rospy.loginfo('New Scene Number: %i', self.scene_number)
				rospy.loginfo('New Part Number: %i', self.part_number)
				self.marker.action = Marker.DELETE 
				self.clicked_point_bool = False
				self.reset_instances = 0
			else:
				#notify user they are done
				self.scene_number = 0
				self.part_number = 0
				rospy.set_param('/study/scene_number', self.scene_number)
				rospy.set_param('/study/part_number', self.part_number)
				rospy.set_param('/study/instruction_number', 0)
				self.marker.action = Marker.DELETE 
				self.clicked_point_bool = False
				self.reset_instances = 0


			return "Recorded"
		else:
			rospy.loginfo('NOT RECORDED: A Point Must Be Clicked First')
			return "Not Recorded"

	def myhook(self):
  		print "shutdown time!"

	def reset_button_callback(self, req):
		print "Reset Button Pressed"
		self.marker.action = Marker.DELETE 
		self.clicked_point_bool = False
		rospy.set_param('/study/instruction_number', 1)
		self.reset_instances = self.reset_instances + 1
		return "Reset"

	def point_callback(self, data):
		#Set boolean to say that a point has been clicked and allow for confimation button and recording
		self.clicked_point_bool = True

		#Set instrution number parameter
		#rospy.set_param('/study/instruction_number', 2)

		#Get point data
		self.picked_point_x = data.point.x
		self.picked_point_y = data.point.y
		self.picked_point_z = data.point.z

		#Get time and duration for clicked point
		self.picked_point_time = data.header.stamp
		self.picked_point_duration = self.picked_point_time - self.start_time

		#Log data
		rospy.loginfo("Clicked Point- x: %f, y: %f, z: %f", self.picked_point_x, self.picked_point_y, self.picked_point_z)

		#Marker Stuff
		self.marker.pose.position.x = self.picked_point_x
		self.marker.pose.position.y = self.picked_point_y
		self.marker.pose.position.z = self.picked_point_z
		self.marker.action = Marker.ADD



def main():
	rospy.init_node('study_master')
	r1 = Report()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException: pass
