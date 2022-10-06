#!/usr/bin/env python2

from __future__ import print_function
from operator import index

from gazebo_msgs.msg import ModelStates
from gazebo_conveyor.srv import ConveyorBeltControl
from sofar_assignment.srv import Conveyor
from std_msgs.msg import Bool
import rospy



global actualPos
global flag
global listNames
global pose
global item, powerOn, powerOff
global pub

#conveyorStopped = 0
powerOn = 15.0
powerOff = 0.0
flag = 0

model_state = None

pub = rospy.Publisher("our_topic", Bool, queue_size = 10)

def BoxTrackerCallback(req):

    global flag, model_state
    global listNames
    global pose, conveyorStopped
    global item, powerOn, powerOff

    flag = 0

    powerOn = 15.0
    powerOff = 0.0
   
    #if (conveyorStopped == 0):
    clientConveyor(powerOn)
    #print("AVVIO CONVEYOR")
    #conveyorStopped = 1
    return "Conveyor started!"   

def modelStateCallback(modelState):

    global model_state, flag, pub
    model_state = modelState
    listNames = model_state.name
	
    nItems = len(listNames)
    #print(nItems)
    #indexBlue = listNames.index('blue_box')
    
    for item in range(0, len(listNames) ):
        pose = model_state.pose[item]
	#print(pose.position.y)
        if (pose.position.y >= 0.538 and  pose.position.y <= 0.548 ): #
            #conveyorStopped = 1
            # chiama service            
            clientConveyor(powerOff)
            print("STOP CONVEYOR", end='\r')
	    if(flag == 0):
            	flag = 1
	    	pub.publish(True)
		print("\n PUBBLICO ")
            

def setup_box_tracker_server():
    
    rospy.init_node("box_tracker_server_node")
    rospy.Service("box_tracker_service", Conveyor, BoxTrackerCallback)
    rospy.Subscriber("gazebo/model_states", ModelStates, modelStateCallback)
    
    print("BOX TRACKER SERVER READY TO PROCESS CLIENT REQUEST")
    rospy.spin()


if __name__ == '__main__':

    rospy.wait_for_service("/conveyor/control")
    clientConveyor = rospy.ServiceProxy("/conveyor/control", ConveyorBeltControl)
    
    setup_box_tracker_server()

    
    

