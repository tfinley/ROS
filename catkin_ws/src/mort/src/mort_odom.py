#!/usr/bin/env python
import roslib; roslib.load_manifest('mort')
import rospy
import tf
from  geometry_msgs.msg import PoseStamped	
from  geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import *
from std_msgs.msg import Int32



def poseCallback(data):
    rospy.loginfo(rospy.get_name() + ": I heard x: %s" % data.pose.pose.position.x)
    rospy.loginfo(rospy.get_name() + ": I heard y: %s" % data.pose.pose.position.y)

def listener():
	rospy.init_node('mort_odom', anonymous=True)
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, poseCallback)
	rospy.spin()	

if __name__ == '__main__':
	listener()
