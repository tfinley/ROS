#!/usr/bin/env python

import roslib; roslib.load_manifest('mort')
import rospy
from std_msgs.msg import String, Int16
from  move_base_msgs.msg import MoveBaseActionResult

#define globals
maxSupply = 5
maxFinished = 5

class Command():
	""" Command MORT """
	def __init__(self):
		self.reached = True
		self.supply = 0 # how many supply are on MORT
		self.finished = 0 # how many finished parts are on MORT
		self.wb1SupplyNeed = 3 # how many supply does WB 1 need
		self.wb1Finished = 1 # how many finished parts are waiting at WB 1
		rospy.Subscriber("move_base/result", MoveBaseActionResult, self.goalReachedCallback)
		rospy.Subscriber("mort/workbench1/supplyNeed", Int16, self.wb1SupplyNeedUpdateCB)
		rospy.Subscriber("mort/workbench1/finished", Int16, self.wb1FinishedUpdateCB)
		self.goal_pub = rospy.Publisher('mort/goal', Int16)
		self.wb1_supply_pub = rospy.Publisher('mort/Workbench1/supplyGiven', Int16)
		self.wb1_taken_pub = rospy.Publisher('mort/Workbench1/finishedTaken', Int16)
		self.goal_msg = Int16()
		self.supply_msg = Int16()
		self.taken_msg = Int16()
		while not rospy.is_shutdown():
			self.updateStatus()
			rospy.loginfo('Decide')
			rospy.sleep(0.5)
			self.decide()

	#FUNCTION: Decision
	def decide(self):
		if self.finished == maxFinished: # a full finished load trumps all
			rospy.loginfo('Decision: 1')
			self.goToShipping()
		elif self.supply >= self.wb1SupplyNeed and self.wb1SupplyNeed > 0: # is there enough supply to fill WB 1?
			rospy.loginfo('Decision: 2')
			self.goToWorkbench1()
		elif self.supply < self.wb1SupplyNeed and self.finished < self.wb1SupplyNeed: # the need for parts is greater than the number of finished parts you have on MORT, then go to the supply
			rospy.loginfo('Decision: 3')
			self.goToSupply()
		elif self.supply <= self.wb1SupplyNeed and self.finished > self.wb1SupplyNeed: # you have little supply and a lot of finished parts on mort, then go to the shipping
			rospy.loginfo('Decision: 4')
			self.goToShipping()
		elif self.supply == 0 and self.finished == 0:
			self.goToSupply()
		else:
			rospy.loginfo('Decision: ELSE')
	
	#FUNCTION: Go To Supply
	def goToSupply(self):
		self.reached = False
		self.goal_msg.data = 2
		for i in range (1,100):
			self.goal_pub.publish(self.goal_msg)
		while not self.reached and not rospy.is_shutdown():
			rospy.loginfo('Driving')
			rospy.sleep(1.0)
		self.supply = maxSupply #load

	#FUNCTION: Go to Workbench 1
	def goToWorkbench1(self):
		self.reached = False
		self.goal_msg.data = 3
		for i in range (1,100):
			self.goal_pub.publish(self.goal_msg)
		while not self.reached and not rospy.is_shutdown():
			rospy.loginfo('Driving')
			rospy.sleep(1.0)
		# -- unload --
		if self.wb1SupplyNeed <= self.supply:
			self.supply_msg.data = self.wb1SupplyNeed # give it what it needs
			self.wb1_supply_pub.publish(self.supply_msg) # tell it what you are giving it
			self.supply = self.supply - self.supply_msg.data # move it off MORT's supply status
		else:
			self.supply_msg.data = self.supply # give it what we have
			self.wb1_supply_pub.publish(self.supply_msg) # tell it what you are giving it
			self.supply = self.supply - self.supply_msg.data # move it off MORT's supply status
		# -- load --
		if self.wb1Finished <= maxFinished - self.finished: # is there room for all finished parts
			self.taken_msg.data = self.wb1Finished # take all
			self.wb1_taken_pub.publish(self.taken_msg) # tell it what you took
			self.finished = self.finished + self.taken_msg.data # update MORT's finished status
		else:
			self.taken_msg.data = maxFinished - self.finished # load what we have room for
			self.wb1_taken_pub.publish(self.taken_msg) # tell it what you took
			self.finished = self.finished + self.taken_msg.data # update MORT's finished status		

	#FUNCTION: Go to Shipping Area
	def goToShipping(self):
		self.reached = False
		self.goal_msg.data = 5
		for i in range (1,100):
			self.goal_pub.publish(self.goal_msg)
		while not self.reached and not rospy.is_shutdown():
			rospy.loginfo('Driving')
			rospy.sleep(1.0)
		self.finished = 0 #unload

	#FUNCTION: Update Status
	def updateStatus(self):
		rospy.loginfo('MORT Supply: %i,', self.supply)
		rospy.loginfo('MORT Finished: %i,', self.finished)

	#CALLBACK: update workbench 1 supply need
	def wb1SupplyNeedUpdateCB(self, msg):
		self.wb1SupplyNeed = msg.data

	#CALLBACK: update workbench 1 finshed parts
	def wb1FinishedUpdateCB(self, msg):
		self.wb1Finished = msg.data

	#CALLBACK: update goal reached status (boolean)
	def goalReachedCallback(self,data):
		if data.status.status == 3: # if it reaches its goal
			self.reached = True
			rospy.loginfo('Goal reached') 		

def main():
	rospy.init_node('mort_command_2_workbenches')
	skynet = Command()
    
if __name__ == '__main__':
    main()
