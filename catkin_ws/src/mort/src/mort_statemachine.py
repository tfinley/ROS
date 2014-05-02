#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state START
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.counter = 0
       

    def execute(self, userdata):
        rospy.loginfo('Starting')
        return 'outcome1'


# define state WAIT
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
        if self.counter < 2:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'
        
# define state SUPPLY_WORKBENCH_1
class SupplyWorkbench1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WORKBENCH 1')
		# go to workbench 1
		# when you are at workbench 1, go to ship
        return 'outcome2'

# define state SUPPLY_WORKBENCH_2
class SupplyWorkbench2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WORKBENCH 2')
		# go to workbench 2
		# when you are at workbench 2, go to ship
        return 'outcome2'

# define state SHIP
class SupplyWorkbench2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SHIP')
		# go to shipping department
		# when you are at ship, go to wait
        return 'outcome1'



def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('START', Start(), 
                               transitions={'outcome1':'WAIT'})
        smach.StateMachine.add('WAIT', Wait(), 
                               transitions={'outcome1':'SUPPLY_WORKBENCH_1', 'outcome2':'SUPPLY_WORKBENCH_2', 'outcome3':'SHIP'})
        smach.StateMachine.add('SUPPLY_WORKBENCH_1', SupplyWorkbench1(), 
                               transitions={'outcome1':'WAIT', 'outcome2':'SHIP'})
        smach.StateMachine.add('SUPPLY_WORKBENCH_2', SupplyWorkbench2(), 
                               transitions={'outcome1':'WAIT', 'outcome2':'SHIP'})
        smach.StateMachine.add('SHIP', Ship(), 
                               transitions={'outcome1':'WAIT'})

		  self.subscriber = rospy.Subscriber('/test', Int16, self.callback)
      #self.image_pub = rospy.Publisher("/mort_goal", Int16)

    def callback(self, data):

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
