# sofar_assignment
================================

Short intro to the assignment



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
Then ```main.cpp``` starts the simulation cycle by calling  ```/spawn_box_service``` to communicate to ```spaw_box_server.py``` to spawn a random colored box through the ```Spawner.srv```:
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

Flowchart explaining how the simulation works

	
	
 ## Code Explanation
 
In the following sections we describe how we implemented the code to obtain the main behaviours of Tiago and Gazebo Simulation.
 ### Box Spawning  :   "```spawn_box_server.py```"
To spawn boxes into the gazebo environment we created this server that is called by the ```main_node``` as soon as no boxes are available.
This node is a client to gazebo's servize : "```gazebo/spawn_urdf_model```", and calls it by passing as arguments: the path of the .URDF file of our boxes (contained in the "```URDF```" folder), and the position where to spawn it. Boxes are then spawned with a random color (blue or green) simply passing the path of the relative .urdf through a randomly generated variable. Moreover we have built these boxes in such a way to be easily grabbable by Tiago's grippers, in terms of weight and dimensions.
	
 
 ### Conveyor movements  :  "```box_tracker_server.py```"
 
 ### Color Detection  :  "```color_detection_server.py```"
I could also put an image here
 
 ### Moving Tiago :   "```move_head.cpp```" & "```main.cpp```"
 

## Possible Improvements
 
use algorithm to detect obj pos. increase number of colors detected, detecting shapes In substitute/addition to colors. 

 
 
 
 
 

