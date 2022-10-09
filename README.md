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
* [Demo simulation](#demo-simulation)
* [Possible Improvements](#possible-improvements)

## Setup

	What and How To install useful thing for this project (ubuntu 18--tiago package--conveyor) and Hown to Run the simulation 
The simulation is built on the ROS (Robot-Operating-Systems) platform, specifically the MELODIC version to be able to have the Tiago Ros package in it. Here the guide for Tiago installation Tiago robot. To use the melodic version it was necessary to work for the project on Ubuntu 18 which can be donwoloaded at Download Ubuntu 18.

The program requires the installation of the following packages and tools for the specific project before it can be launched:



## Gazebo Environment

This image shows the simulation environment we created for this assignment, consisting of Tiago placed at end of a conveyor belt with coloured bins to its sides where to put relative boxes:
<p>
<img src="https://github.com/claudio-dg/sofar_assignment/blob/main/images/Gazebo_world.png?raw=true" width="700"/>
<p>
	
## Project structure and behaviour description

### Flowchart
	
<p>
<img src="https://github.com/claudio-dg/sofar_assignment/blob/main/images/Tiago%20Flowchart.png" width="700"/>
<p>

### UML
	
<p>
<img src="https://github.com/claudio-dg/sofar_assignment/blob/main/images/UML.jpeg" width="700"/>
<p>
	
 ### Behaviour description  :
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


 ### RQT graph  : ### 
This is the schema of the project in ```sofar_assignment``` and the interaction between the nodes. ROS can generate this graph by the command:

```bash
$ rosrun rqt_graph rqt_graph
```

<p>
<img src="https://github.com/claudio-dg/sofar_assignment/blob/main/images/rosgraph.png" width="1000"/>
<p>
	
	
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
 To obtain Color Detection we made use of ```OpenCV``` Computer Vision libraries and created this server that is called by the ```main``` when the conveyor is stopped. It subscribes to Tiago's camera sensor ```/xtion/rgb/image_raw``` in order to have real time images from Tiago's point of view. This image is converted into a CV format through the ```bridge.imgmsg_to_cv2``` function, then the RGB components are converted into HSV (hue-saturation-value) with ```cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)``` function.
To distinguish color we then created masks choosing some ranges for these HSV components to be applied to the image retrieved from Tiago's camera. The code will then check wether it recognises one of these colors (blue-green) in the image and will also output a pop-up showing the detected color as shown in the following image:
		
<p>
<img src="https://github.com/claudio-dg/sofar_assignment/blob/main/images/color_detect.png?raw=true" width="600"/>
<p>
		
The code of what previosuly described is the following:
```bash
def callbackFunc(req):

	global img
	

	try:
		imageFrame = bridge.imgmsg_to_cv2(img, "passthrough")
	except CvBridgeError as e:
		rospy.logerr("Cv bridge error: {0}".format(e))
	
	# Convert the imageFrame in BGR(RGB color space) to HSV(hue-saturation-value) color space	
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

	
	# Set range for blue color and define mask
	blue_lower = np.array([94, 80, 2], np.uint8)
	blue_upper = np.array([120, 255, 255], np.uint8)
	blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
	


	# Set range for green color and define mask
	green_lower = np.array([25, 52, 72], np.uint8)
	green_upper = np.array([102, 255, 255], np.uint8)
	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
	

	# Morphological Transform, Dilation for each color and bitwise_and operator between imageFrame and mask determines to detect only that particular color
	kernal = np.ones((5, 5), "uint8")

	# For blue color
	blue_mask = cv2.dilate(blue_mask, kernal)
	res_blue = cv2.bitwise_and(imageFrame, imageFrame,
							mask = blue_mask)
	
	# For green color
	green_mask = cv2.dilate(green_mask, kernal)
	res_green = cv2.bitwise_and(imageFrame, imageFrame,
								mask = green_mask)
	
	# Creating contour to track green color
	contours, hierarchy = cv2.findContours(green_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)[-2:]
	
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(0, 255, 0), 2)
			
			cv2.putText(imageFrame, "Green Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX,
						1.0, (0, 255, 0))
			# Program Termination
			cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
			
			if cv2.waitKey(10) & 0xFF == ord('q'): 
				cap.release()
				cv2.destroyAllWindows()

			return True

	# Creating contour to track blue color
	contours, hierarchy = cv2.findContours(blue_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)[-2:]
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(255, 0, 0), 2)
			
			cv2.putText(imageFrame, "Blue Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX,
						1.0, (255, 0, 0))
			# Program Termination
			cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
			if cv2.waitKey(10) & 0xFF == ord('q'): ##################
				cap.release()
				cv2.destroyAllWindows()

			return False

```
 ### Moving Tiago :   "```move_head.cpp```" & "```main.cpp```"
 To move Tiago we made two different codes, but the genereal idea is to have some predefined motions to  move each joint, knowing in advance what the final position of the boxes to detect and grab will be on the conveyor belt.
"```move_head.cpp```" create a ROS client to head controller ("```/head_controller/point_head_action```") and then assigns a relative position to reach with respect to the current point of view of Tiago, in order to have a better centered sight of the boxes and facilitate the color detection task.
```bash		
void moveHead(void){

    geometry_msgs::PointStamped pointStamped;
    pointStamped.header.frame_id = "/xtion_rgb_optical_frame"; 
    double x = 0.0;
    double y = 0.0;
    double Z = 1.0;

    // position is relative so taken at the current image view
    pointStamped.point.x = 0.0 * Z;
    pointStamped.point.y = 0.7 * Z; // positive down, negative up
    pointStamped.point.z = Z; 


    //build the action goal
    control_msgs::PointHeadGoal goal;
    //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
    goal.pointing_frame = "/xtion_rgb_optical_frame";
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.25;
    goal.target = pointStamped;

    pointHeadClient->sendGoal(goal);
    ros::Duration(0.5).sleep();

}
```
The ```main``` instead, creates Action clients to ```/arm_controller/follow_joint_trajectory``` ```/torso_controller/follow_joint_trajectory``` ```/parallel_gripper_controller/follow_joint_trajectory``` to respectively move the arm, torso and gripper.
Regarding the torso movement we just made Tiago go up a bit to make sure he was able to reach the object and not collide with aniìythin in the environment. (please note that in the simulation tiago is a bit far from the conveyor, since we noticed that the hit boxes of this last presented some issues being bigger than the actual dimension of it, so to avoid crushing Tiago this distance from the conveyor, and torso movement were required).
For the grippers we just specified the closing/opening by setting the joint position to minimun or maximum:
```bash		
void waypoints_closeGripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
  goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.0;
  goal.trajectory.points[index].positions[1] = 0.0;

  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
  ROS_INFO("closing gripper");
 
}
void closeGripper()
{
//Create an torso controller action client to move the TIAGo's gripper
  gripper_control_client_Ptr GripperClient;
  createGripperClient(GripperClient);

  // Generates the goal for the TIAGo's torso
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_closeGripper_goal(gripper_goal);

  // Sends the command to start the given trajectory 1s from now
  gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  GripperClient->sendGoal(gripper_goal);
  ROS_INFO("grabbing the object");

  // Wait for four seconds instead of trajectory execution, because here the 0.0 position of grippers will never be reached due to box presence
  ros::Duration(4).sleep();

}
```
In particular for the "closing gripper" movement we had to put a ```"ros::Duration(4).sleep();"``` to wait for the trajectory execution instead of using the usual:
```bash
while(!(GripperClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }
```
beacuse in the case of "closing gripper", the robot was never able to actually reach the desired configuration due to the presence of the object between grippers, therefore by putting this "wait", we managed to obtain a secure grab of the objects.
In the end, regarding arm movements, we created several waypoints for a certain set of movements that are: PregraspMovement, PostgraspMovements (towards blue bin or green bin) and PostDropMovements to return to a a starting position from which the robot was able to reproduce the PregraspMovement again.
Each of these motions presents several waypoints both to avoid collision with possible elemnts in the environment, both to allow a correct execution of the tasks avoiding, for instance, too fast movements in the "PostGrasp" step that would cause the box to eventually slip out of Tiago's gripper. For sake of simplicity the code of this part is not shown here in this ```README```, but can be found in ```src/main.cpp```.

## Demo Simulation

[<iframe width="560" height="315" src="https://www.youtube.com/embed/BwsuGeH5LvY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>](https://github.com/claudio-dg/sofar_assignment/blob/main/README.md)
		
## Possible Improvements

In this project, we designed a strategy for solving the 3D object color detection problem for the robot named Tiago which was able to recognize, pick up and release boxes according to their colors in their specific place which can be done in many other ways. So instead of getting manually the waypoints related to the movements of the robot arm certainly in a way that the parameters are clearly chosen, therefore, no collisions with objects will occur.

**Moveit** contains other tutorials that could be also used for the same purpose which are responsible of the movements so that the robot will be able to autonomously detect the position of the object to grab, defining the necessary arm movements in order to grab it and this can be counted as an improved option to our project.

While for the detection of the objects, Tiago is equipped with an RGB-D camera that was able to catch two colors of the boxes (green and blue) spawned on the conveyor belt, whereas a possible improvement it could be or allowing Tiago to grasp the object and release it relying on the shape or both the color and the shape of the object or by giving the ability for the robot to distinguish more colors, simply by adding new "color masks" in the ```color_detection_server.py``` script. 

 
 
 
 
 

