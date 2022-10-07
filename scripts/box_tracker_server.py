#!/usr/bin/env python2


"""
- Module: box_tracker_server.py
- Authors: D. Bruzzo, C. Del Gaizo, P. Saade, C. Tsague
- 
- Subscribes to:
-		"gazebo/model_states" : to take trace box's position
-
- Publishes to:
-		"/our_topic" : to publish when the box has reached the position in which the conveyor belt should be stopped
-
- Client:
-	"/conveyor/control" :  to start and stop conveyor
-
- Server:
-	"/box_tracker_service"
-
-
- Description:
-
- This node function as a server that the main.cpp node calls it when the conveyor needs to be activated.
- This node also controls when the box reaches the right position at the end of the conveyor belt and it stops it. 
- At this time also publishes a message to communicate to the main node that the box now need to be taken/analized and so gives back control to main.cpp. 
-
-
"""


from __future__ import print_function
from operator import index

from gazebo_msgs.msg import ModelStates
from gazebo_conveyor.srv import ConveyorBeltControl
from sofar_assignment.srv import Conveyor
from std_msgs.msg import Bool
import rospy


# global variable declaration
global flag
global listNames
global pose
global item, powerOn, powerOff
global pub

# global variable initialization
powerOn = 15.0
powerOff = 0.0
flag = 0

model_state = None

# publisher initialization
pub = rospy.Publisher("our_topic", Bool, queue_size = 10)

def BoxTrackerCallback(req):

    """ 
    Server callback executed when the main.cpp calls the box server client. 
    This function starts the conveyor belt by calling its client.
    
    """
    global flag, model_state
    global listNames
    global pose, conveyorStopped
    global item, powerOn, powerOff

    flag = 0

    powerOn = 15.0
    powerOff = 0.0
 
    # calling the client of conveyor for activating it
    clientConveyor(powerOn)
  
    return "Conveyor started!"   

def modelStateCallback(modelState):

    """ 
    Callback that checks the position of the box. 
    When it triggers a condition it calls the client to stop the conveyor.
    
    """
	
    global model_state, flag, pub
    model_state = modelState
    listNames = model_state.name
	
    nItems = len(listNames)
    
    #check every item in the list of gazebo's objects
    for item in range(0, len(listNames) ):
        pose = model_state.pose[item]
		
	#if the item is between that coordinates  we need to stop the conveyor
        if (pose.position.y >= 0.538 and  pose.position.y <= 0.548 ):                       
            clientConveyor(powerOff)
            print("STOP CONVEYOR", end='\r')
	    if(flag == 0):
            	flag = 1
	    	pub.publish(True)
		print("\n PUBBLICO ")
            

def setup_box_tracker_server():
	
    """ 
    Function called to setup the server and also subscribe to the model_states topic
	
    """
    # initialize the node
    rospy.init_node("box_tracker_server_node")
    rospy.Service("box_tracker_service", Conveyor, BoxTrackerCallback)
    rospy.Subscriber("gazebo/model_states", ModelStates, modelStateCallback)
    
    print("BOX TRACKER SERVER READY TO PROCESS CLIENT REQUEST")
    rospy.spin()


if __name__ == '__main__':

    rospy.wait_for_service("/conveyor/control")
    clientConveyor = rospy.ServiceProxy("/conveyor/control", ConveyorBeltControl)
    
    setup_box_tracker_server()

    
    

