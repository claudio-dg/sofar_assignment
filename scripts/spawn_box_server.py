#!/usr/bin/env python2


import rospy, tf
from random import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates 
from geometry_msgs.msg import Point, Quaternion, Pose
from sofar_assignment.srv import Spawner



global boxN, itemName, blue_path, green_path, item_path, orient, initial_pose, rand_flag

boxN = 1


def spawn_model(req):

    global boxN, itemName, blue_path, green_path, item_path, orient, initial_pose, rand_flag

    rand_flag = int(random()*100)

    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0.4
    initial_pose.position.z = 1.3

    blue_path = '/home/piya/Assignment_ws/src/sofar_assignment/urdf/blue_box.urdf' 
    green_path = '/home/piya/Assignment_ws/src/sofar_assignment/urdf/green_box.urdf'
    #blue_path = rospkg.RosPack().get_path('sofar_assignment')+'/urdf/blue_box.urdf' 
    #green_path = rospkg.RosPack().get_path('sofar_assignment')+'/urdf/green_box.urdf' 

    orient = Quaternion(0,0,0,0)


    if(rand_flag % 2):
        #prendo blu
        itemName = "blue_box_" + str(boxN)
        item_path = blue_path
    else:
        #prendo green
        itemName = "green_box_" + str(boxN)
        item_path = green_path

    boxN += 1

    rospy.wait_for_service("gazebo/spawn_urdf_model")
    spawnClient = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

    spawnClient(itemName, open(item_path, 'r').read(), "", initial_pose, "world")
    print("Spawned : ", itemName)

    return True

def setup_spawn_server():
    
    rospy.init_node("spawn_box_server_node")
    rospy.Service("spawn_box_service", Spawner, spawn_model)

    print("SPAWN SERVER READY TO PROCESS CLIENT REQUEST")
    rospy.spin()



if __name__=='__main__':

    setup_spawn_server()
   
