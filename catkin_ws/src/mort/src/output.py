#!/usr/bin/env python


# Every python controller needs these lines
import roslib; roslib.load_manifest('mort')
import rospy
import numpy
import os


from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16

#Define global vars
glbwork1in = [0,0,0]		#workbench1 queue
glbwork2in = [0,0,0]		#workbench2 queue
glbwork1out = [0,0]		#workbench1 output
glbwork2out = [0,0]		#workbench2 output
coff = 0			#Interrupt Request
tarr = 1			#Current Target
robot = [0,0]			#Robot Payload

##Updates
def wk1incase(data):
	global glbwork1in
	glbwork1in = [round(elem,2) for elem in data.data] #Import & remove float error
	#print(len(glbwork1in))
	displ()

def wk1outcase(data):
	global glbwork1out
	glbwork1out = [round(elem,2) for elem in data.data] #Import & remove float error
	#print(len(glbwork1in))
	displ()

def wk2incase(data):
	global glbwork2in
	glbwork2in = [round(elem,2) for elem in data.data] #Import & remove float error
	#print(len(glbwork1in))
	displ()

def wk2outcase(data):
	global glbwork2out
	glbwork2out = [round(elem,2) for elem in data.data] #Import & remove float error
	#print(len(glbwork1in))
	displ()

def coffeee(data):
	global coff
	coff = coff+data.data
	displ()

def tarr(data):
	global tarr
	tarr = data.data
	displ()

def robott(data):
	global robot
	robot = [round(elem,2) for elem in data.data] #Import & remove float
	displ()

def coffeee2(data):
	global coff	
	coff=0
	displ()

def displ():
	os.system('clear')
	##interrupt request
	
	#Populate robot payload str
	robotpaystr = ''
	for i in range(0,2):
		if robot[i]==1:
			robotpaystr = robotpaystr + ' |U|'
		elif robot[i]==1.1:
			robotpaystr = robotpaystr + ' |F|'
		elif robot[i]==1.01:

			robotpaystr = robotpaystr + '|  U  |'
			break
		elif robot[i]==1.11:
			robotpaystr = robotpaystr + '|  F  |'
			break
		else:
			robotpaystr = robotpaystr + ' ___'



	global coff,robot
	if coff>7:
		print('COFFEE MACHINE: REQUEST ACCEPTED - CHANGING COURSE\n')
	elif coff>0:
		print('COFFEE MACHINE: REQUEST RECIEVED - WILL CHANGE COURSE AFTER TASK\n')
	else:
		print('COFFEE MACHINE:\n')
	if tarr==5:
		print ('  <robot driving to this location with payload [' + robotpaystr + ' ] >')
	else:	
		print('')

	#######Raw Material----------------------------------
	print('Raw Material Bin')
	if tarr==1:
		print ('  <robot driving to this location with payload [' + robotpaystr + ' ] >')
	else:	
		print('')
		
	#######WORKER 1----------------------------------
	##Outputs text to show progress
	#Queue string
	dispstr = 'Queue:   '
	skip = 0  #Skips a loop iteration
	for i in range (0,len(glbwork1in)):
		if glbwork1in[i] == 0:
			dispstr = dispstr + ' ___'
		elif skip ==1:  #Skip this iteration to avoid large box error
			skip = 0;
		elif glbwork1in[i] == 1:
			dispstr = dispstr + ' |U|'
		elif glbwork1in[i] == 1.01 and glbwork1in[i+1] == 1.01:
			dispstr = dispstr + ' |  U  |'
			skip = 1
		elif glbwork1in[i] == 1.1:
			dispstr = dispstr + ' |F|'
		elif glbwork1in[i] == 1.11 and glbwork1in[i+1] == 1.11:
			dispstr = dispstr + ' |  F  |'            
	print('\nWorkbench 1: U=unfinished, F=finished')
	print (dispstr)

	#Output string
	dispstr = 'Output:  '
	skip = 0  #Skips a loop iteration
	for i in range (0,len(glbwork1out)):
		if glbwork1out[i] == 0:
			dispstr = dispstr + ' ___'
		elif skip ==1:  #Skip this iteration to avoid large box error
			skip = 0;
		elif glbwork1out[i] == 1.1:
			dispstr = dispstr + ' |F|'
		elif glbwork1out[i] == 1.11:
			dispstr = dispstr + ' |  F  |'
			skip = 1
	print (dispstr)
	if tarr==2:
		print ('  <robot driving to this location with payload [' + robotpaystr + ' ] >')
	else:	
		print('')

	#######WORKER 2----------------------------------
	print('')
	dispstr = 'Queue:   '
	skip = 0  #Skips a loop iteration
	for i in range (0,len(glbwork2in)):
		if glbwork2in[i] == 0:
			dispstr = dispstr + ' ___'
		elif skip ==1:  #Skip this iteration to avoid large box error
			skip = 0;
		elif glbwork2in[i] == 1:
			dispstr = dispstr + ' |U|'
		elif glbwork2in[i] == 1.01 and glbwork2in[i+1] == 1.01:
			dispstr = dispstr + ' |  U  |'
			skip = 1
		elif glbwork2in[i] == 1.1:
			dispstr = dispstr + ' |F|'
		elif glbwork2in[i] == 1.11 and glbwork2in[i+1] == 1.11:
			dispstr = dispstr + ' |  F  |'            
	print('\nWorkbench 2: U=unfinished, F=finished')
	print (dispstr)

	#Output string
	dispstr = 'Output:  '
	skip = 0  #Skips a loop iteration
	for i in range (0,len(glbwork2out)):
		if glbwork2out[i] == 0:
			dispstr = dispstr + ' ___'
		elif skip ==1:  #Skip this iteration to avoid large box error
			skip = 0;
		elif glbwork2out[i] == 1.1:
			dispstr = dispstr + ' |F|'
		elif glbwork2out[i] == 1.11:
			dispstr = dispstr + ' |  F  |'
			skip = 1
	print (dispstr)
	if tarr==3:
		print ('  <robot driving to this location with payload [' + robotpaystr + ' ] >')
	else:	
		print('')
	print('')

	#######Finished Goods----------------------------------
	print('Finished Goods Bin')
	if tarr==4:
		print ('  <robot driving to this location with payload [' + robotpaystr + ' ] >')
	else:
		print('')



if __name__ == '__main__':
	rospy.init_node('scrnout')
	os.system('clear')
	rospy.Subscriber('work1in',Float32MultiArray,wk1incase)
	rospy.Subscriber('work1out',Float32MultiArray,wk1outcase)
	rospy.Subscriber('work2in',Float32MultiArray,wk2incase)
	rospy.Subscriber('work2out',Float32MultiArray,wk2outcase)
	rospy.Subscriber('coffee',Int16,coffeee)
	rospy.Subscriber('coffeegot',Int16,coffeee2)
	rospy.Subscriber('target2',Int16,tarr)
	rospy.Subscriber('robotpayload',Float32MultiArray,robott)
	rospy.spin()
