#!/usr/bin/env python
#Worker Node

# Every python controller needs these lines
import roslib; roslib.load_manifest('mort')
import rospy
import math
import numpy
import time
import random

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16

glbwork1in = [0,0,0]		#workbench1 queue
glbwork1out = [0,0]		#workbench1 output
latch1 = 0
latch2 = 0

##UPDATE GLOBAL VARIABLE
def wk1incase(data):	
	global glbwork1in
	inp = [round(elem,2) for elem in data.data] #Import & remove float error
	occin = int(sum(glbwork1in))
	glbwork1in[occin:occin+len(inp)]=inp
	#if len(glbwork1in)>3:
	#	del(glbwork1in[3-1:len(glbwork1in)])
	wk1in.publish(data=glbwork1in)

##UPDATE GLOBAL VARIABLE
def wk1outcase(data):
	global glbwork1out,glbwork1in
	outp = data.data #Import & remove float error
	occout = int(sum(glbwork1out))
	if outp == 999:
		glbwork1in = [0,0,0]		#workbench1 queue
		glbwork1out = [0,0]		#workbench1 output
	elif outp ==1:
		print (occout)
		glbwork1out[occout-1]=0
	elif outp ==2:
		glbwork1out= [0,0]
	else:
		print('error')

	wk1in.publish(data=glbwork1in)
	wk1out.publish(data=glbwork1out)



##START WORK
def decay():
	global glbwork1in, glbwork1out
	
	while not rospy.is_shutdown():

		#Keep working when unfinished goods
		if glbwork1in[0] == 1 or glbwork1in[0] ==1.01: 
			#Perform work on object to turn into finished good
			if glbwork1in[0]>0 and glbwork1in[0]<1.1:
				rospy.sleep(30*random.random()+90)
				glbwork1in[0]=glbwork1in[0]+.1 #First Cell - all cases
				if glbwork1in[0] == 1.11: #If large object, iterate second cell
					glbwork1in[1]=glbwork1in[0]
			wk1in.publish(data=glbwork1in)



		#Move Goods Over
		if (glbwork1out[1]==0 and glbwork1in[0]==1.1) or (glbwork1out[0]==0 and glbwork1in[0]==1.11):
			#If there is finished goods and space to put them - move it over
			occout = int(sum(glbwork1out))
			#print(occout)
			#print(glbwork1in)
			#print(glbwork1out)
			if occout<2 and glbwork1in[0]==1.1:    #Small box case
				tempin = glbwork1in   #move over to temp to avoid confusion with other nodes		
				glbwork1out[occout]=1.1  #Move over to finished goods
				del tempin[0]  #delete from queue
				tempin = tempin+[0] #add empty space at end of list
				glbwork1in = tempin #merge back in
			elif occout==0 and glbwork1in[0]==1.11:    #LARGE box case
				glbwork1out[occout]=glbwork1in[0]  #Move over to finished goods space 1
				glbwork1out[occout+1]=glbwork1in[1]
				tempin = glbwork1in   #move over to temp to avoid confusion with other nodes
				del tempin[0:2]  #delete from queue
				tempin = tempin+[0,0] #add empty space at end of string
				glbwork1in = tempin #merge back in        
			wk1in.publish(data=glbwork1in)
			wk1out.publish(data=glbwork1out)



if __name__ == '__main__':
	rospy.init_node('worker1node')
	wk1in = rospy.Publisher('work1in',Float32MultiArray)
	wk1out = rospy.Publisher('work1out',Float32MultiArray)
	
	rospy.Subscriber('work1inpkt',Float32MultiArray,wk1incase)
	rospy.Subscriber('work1outpkt',Int16,wk1outcase)

	decay()
	rospy.spin()
