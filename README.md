# sofar_assignment
================================

Introduction:
-------------
The main purpose of this assignment (#14) is to build a software architecture (Ros 1 in our case) and a simulation environment containing a conveyor belt where 3D boxes are spawned one after the other and detected depending on their colors. Accordingly a robot manipulator (Tiago) is able to estimate their color through an RGB-D camera, grasp them box and put each box in its specific bin.

Authors: D. Bruzzo C. Del Gaizo P. Saade C. Tsague

Table of contents
----------------------

* [Setup](#setup)
* [Gazebo Environment](#gazebo-environment)
* [Project structure and behaviour description](#project-structure-and-behaviour-description)
* [Code Explanation](#code-explanation)
* [Possible Improvements](#possible-improvements)

## Setup

	What and How To install useful thing for this project (ubuntu 18--tiago package--conveyor) and Hown to Run the simulation 
The simulation is built on the ROS (Robot-Operating-Systems) platform, specifically the MELODIC version to be able to have the Tiago Ros package in it. Here the guide for Tiago installation Tiago robot. To use the melodic version it was necessary to work for the project on Ubuntu 18 which can be donwoloaded at Download Ubuntu 18.

The program requires the installation of the following packages and tools for the specific project before it can be launched:



## Gazebo Environment

Image of the envirnoment/world
<p>
<img src="https://github.com/claudio-dg/sofar_assignment/blob/main/images/Gazebo_world.png?raw=true" width="700"/>
<p>
	
## Project structure and behaviour description

we show rqt graph and uml explaing the communication among nodes
	
The idea of this project is to have a central node, that is ```main.cpp```, which occupies controlling all the other nodes. Through ```services``` and ```messages``` it temporizes the right time when to call the correct node.
At first, when the world is launched, the servers are launched and the first node that is called is ```move_head.cpp``` which moves down Tiago's head.
Then ```main.cpp``` starts the simulation cycle by moving the torso up a bit and calling  ```/spawn_box_service``` to communicate to ```spaw_box_server.py``` to spawn a random colored box through the ```Spawner.srv```:
```cpp
---
bool spawned

```
Once the server returns the reply, which means that the box is spawned, main.cpp calls the ```/box_tracker_service``` to tell ```box_tracker_server.py``` to sart moving the conveyor belt through ```Conveyor.srv```:
```cpp

---
string conveyorStatus

```
This is not enough to main.cpp to enter in the next step of the cycle. It needs to wait for ```box_tracker_server.py``` for publishing that the box has reached the final position on the conveyor belt and this last one has been stopped. This ```publish/subscribe``` communication is done through the topic ```our_topic``` on ```Bool``` msg. The callback in main.cpp, when called, changes the value needed to enter in the next step. 
Then the last communication is between main.cpp and the node ```color_detection_server.py```. Through the service ```/color_detect_service``` the server communicates the color of the box to the main node on ```DetectColor.srv```:
```cpp
# sending the color: 0 = blue, 1 = green

---
bool color 
```
This server is subbed to ```/xtion/rgb/image_raw``` topic in order to continuously update the image on which perform the color detection.
Then ```main.cpp``` knows in which bin put the box based on the color and starts moving Tiago's arm toward the object to grab it.


 ### Behaviour description  : ### 

Flowchart explaining how the simulation works & VIDEO DEMO

	
	
 ## Code Explanation
 
In the following sections we describe how we implemented the code to obtain the main behaviours of Tiago and Gazebo Simulation.
 ### Box Spawning  :   "```spawn_box_server.py```"
To spawn boxes into the gazebo environment we created this server that is called by the ```main_node``` as soon as no boxes are available.
This node is a client to gazebo's service : "```gazebo/spawn_urdf_model```", and calls it by passing as arguments: the path of the .URDF file of our boxes (contained in the "```URDF```" folder), and the position where to spawn it. Boxes are then spawned with a random color (blue or green) simply passing the path of the relative .urdf through a randomly generated variable. Moreover we have built these boxes in such a way to be easily grabbable by Tiago's grippers, in terms of weight and dimensions.
```bash
def spawn_model(req):
	
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
```
 
 ### Conveyor movements  :  "```box_tracker_server.py```"
To communicate with the Conveyor belt we created this server that is called by the ```main_node``` as soon as a new box is spawned. This node is a client to Conveyor's Plugin service "```/conveyor/control```", and calls it by passing the desired power of the conveyor to make it move or stop it when necessary. In addition to that it subscribes to Gazebo's topic "```gazebo/model_states```" to be aware at each time of the position of each model in the environment: thanks to this we managed to implement the algorithm for which the conveyor is stopped when a box reaches its final position on the conveyor, allowing it to be grabbed by Tiago.
```bash
def modelStateCallback(modelState):
	
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
```	
The here shown code refers to the callback of "```gazebo/model_states```" topic: here the algorithm is based on a continous check of the elements existing in the environment through the array of names; when the y-coordinate of one of them enters in a certain range (i.e. 0.538 < y < 0.548), it means that a box has reached the final position, therefore the conveyor is stopped and a msg is published on ```our_topic``` to advertise the ```main``` node
								 
								 
 ### Color Detection  :  "```color_detection_server.py```"
I could also put an image here
 
 ### Moving Tiago :   "```move_head.cpp```" & "```main.cpp```"
 

## Possible Improvements

In this project, we designed a strategy for solving the 3D object color detection problem for the robot named Tiago which was able to recognize, pick up and release boxes according to their colors in their specific place which can be done in many other ways. So instead of getting manually the waypoints related to the movements of the robot arm certainly in a way that the parameters are clearly chosen, therefore, no collisions with objects will occur.

**Moveit** contains other tutorials that could be also used for the same purpose which are responsible of the movements so that the robot will be able to autonomously detect the position of the object to grab, defining the necessary arm movements in order to grab it and this can be counted as an improved option to our project.

While for the detection of the objects, Tiago is equipped with an RGB-D camera that was able to catch two colors of the boxes (green and blue) spawned on the conveyor belt, whereas a possible improvement it could be or allowing Tiago to grasp the object and release it relying on the shape or both the color and the shape of the object or by giving the ability for the robot to distinguish more colors, simply by adding new "color masks" in the ```color_detection_server.py``` script. 

 
 
 
 
 

