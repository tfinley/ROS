#!/usr/bin/env python
import roslib; roslib.load_manifest('mort')
import rospy
import tf
from  geometry_msgs.msg import PoseStamped	
from visualization_msgs.msg import *
from std_msgs.msg import Int16




def publishGoalMarker(goal):

	marker_pub = rospy.Publisher('/mort/goal_marker', visualization_msgs.msg.Marker)
	marker = visualization_msgs.msg.Marker()
	print "publishing markers"
	marker.header.frame_id = 'map';
	marker.pose = goal.pose
	marker.header.stamp = rospy.Time.now()
	marker.ns = "mortgoal";
	marker.action = marker.ADD
	marker.type = marker.SPHERE
	marker.color.a = 1.0;
	marker.color.g = 1.0;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	print marker
	marker_pub.publish(marker);

def goalCallback(goalNumber):
	'''
	convert an interger goal to a geometry_msgs.msg.PoseStamped goal
	'''
	pose = PoseStamped()
	pose.header.frame_id = 'map'
	if goalNumber.data == 1: # close door
		pose.pose.position.x = -3.636
		pose.pose.position.y = -1.351
		pose.pose.position.z = 0.0    
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = 0.902232927368
		pose.pose.orientation.w = -0.431249051909
	elif goalNumber.data == 5: # fridge
		pose.pose.position.x = -1.6062
		pose.pose.position.y = 2.9547
		pose.pose.position.z = 0.0    
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = 0.453129839787
		pose.pose.orientation.w = 0.891444529006
	elif goalNumber.data == 2: # matts desk
		pose.pose.position.x = 1.8161
		pose.pose.position.y = -1.1600
		pose.pose.position.z = 0.0    
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = 0.462832849886
		pose.pose.orientation.w = 0.886445572535
	elif goalNumber.data == 3: # under tv
		pose.pose.position.x = 1.6162
		pose.pose.position.y = -4.8752
		pose.pose.position.z = 0.0    
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = -0.28562332892
		pose.pose.orientation.w = 0.958341960877
	elif goalNumber.data == 4: # far door
		pose.pose.position.x = 5.9445
		pose.pose.position.y = -8.3671
		pose.pose.position.z = 0.0    
		pose.pose.orientation.x = 0.0
		pose.pose.orientation.y = 0.0
		pose.pose.orientation.z = 0.893372739203
		pose.pose.orientation.w = -0.449316312689
	else:
		print('no set goal for input')

	pub = rospy.Publisher('move_base_simple/goal', PoseStamped)
	pose.header.stamp = rospy.Time.now()
	pub.publish(pose)
	publishGoalMarker(pose)

def listener():
	rospy.init_node('mort_goal_manager', anonymous=True)
	rospy.Subscriber("target2", Int16, goalCallback)
	rospy.spin()	

if __name__ == '__main__':
	listener()