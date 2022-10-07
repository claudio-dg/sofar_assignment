#!/usr/bin/env python2

"""
- Module: spawn_box_server.py
- Authors: D. Bruzzo, C. Del Gaizo, P. Saade, C. Tsague
-
- Client:
-	"gazebo/spawn_urdf_model" :  to spawn the box
-
- Server:
-	"/spawn_box_servicee"
-
-
- Description:
-
- This node function as a server that the main.cpp node calls it by first to start the whole chain of simulation.
- This node calls the client to spawn the box. 
-
"""

import rospy, tf
from random import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates 
from geometry_msgs.msg import Point, Quaternion, Pose
from sofar_assignment.srv import Spawner


# global variable declaration
global boxN, itemName, blue_path, green_path, item_path, orient, initial_pose, rand_flag

# global variable initialization
boxN = 1


def spawn_model(req):

    """ 
    Server callback executed when the main.cpp calls the spawn_box_server client. 
    This function spawns the box by calling gazebo spawner client.
    
    """
    global boxN, itemName, blue_path, green_path, item_path, orient, initial_pose, rand_flag

    # get a random integer between 0 and 100
    rand_flag = int(random()*100)

    # set the spawning position
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0.4
    initial_pose.position.z = 1.3

    # declare the urdf path
    blue_path = '/home/piya/Assignment_ws/src/sofar_assignment/urdf/blue_box.urdf' 
    green_path = '/home/piya/Assignment_ws/src/sofar_assignment/urdf/green_box.urdf'

    # set the spawning orientation
    orient = Quaternion(0,0,0,0)


    if(rand_flag % 2):
        #blue chosen
        itemName = "blue_box_" + str(boxN)
        item_path = blue_path
    else:
        #green chosen 
        itemName = "green_box_" + str(boxN)
        item_path = green_path

    boxN += 1

    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawnClient = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    

    # calling the client of spawner 
    spawnClient(itemName, open(item_path, 'r').read(), "", initial_pose, "world")
    print("Spawned : ", itemName)

    return True

def setup_spawn_server():
    
    """ 
    Function called to setup the server
	
    """
    
    rospy.init_node("spawn_box_server_node")
    rospy.Service("spawn_box_service", Spawner, spawn_model)

    print("SPAWN SERVER READY TO PROCESS CLIENT REQUEST")
    rospy.spin()



if __name__=='__main__':

    setup_spawn_server()
   
