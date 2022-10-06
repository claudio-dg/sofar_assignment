
/** \author . */

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

#include <sofar_assignment/Spawner.h>
#include<sofar_assignment/Conveyor.h>
#include<sofar_assignment/DetectColor.h>

#include<std_msgs/Bool.h>



// Our Action interface type for moving TIAGo's arm, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

// Our Action interface type for moving TIAGo's torso, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_control_client;
typedef boost::shared_ptr< torso_control_client>  torso_control_client_Ptr;

// Our Action interface type for moving TIAGo's torso, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;
typedef boost::shared_ptr< gripper_control_client> gripper_control_client_Ptr;


// Create a ROS action client to move TIAGo's arm
void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

// Create a ROS action client to move TIAGo's torso
void createTorsoClient(torso_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to torso controller ...");

  actionClient.reset( new torso_control_client("/torso_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for torso controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the torso_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createTorsoClient: torso controller action server not available");
}


// Create a ROS action client to move TIAGo's gripper
void createGripperClient(gripper_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to gripper controller ...");

  actionClient.reset( new gripper_control_client("/parallel_gripper_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for gripper controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the gripper_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createGripperClient: gripper controller action server not available");
}


// Generates a simple trajectory with two waypoints to move TIAGo's arm 
void waypoints_arm_pregrasp(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(3);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.07;
  goal.trajectory.points[index].positions[1] = 0.01;
  goal.trajectory.points[index].positions[2] = -3.46;
  goal.trajectory.points[index].positions[3] = 1.45;
  goal.trajectory.points[index].positions[4] = 0.04;
  goal.trajectory.points[index].positions[5] = 0.0;
  goal.trajectory.points[index].positions[6] = 0.0;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
  
  // Second trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.07;
  goal.trajectory.points[index].positions[1] = 0.14;
  goal.trajectory.points[index].positions[2] = -1.43;
  goal.trajectory.points[index].positions[3] = 1.45;
  goal.trajectory.points[index].positions[4] = 2.07;
  goal.trajectory.points[index].positions[5] = -1.39;
  goal.trajectory.points[index].positions[6] = 1.70;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
  

// Third trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.83;
  goal.trajectory.points[index].positions[1] = 0.01;
  goal.trajectory.points[index].positions[2] = -1.45;
  goal.trajectory.points[index].positions[3] = 0.89;
  goal.trajectory.points[index].positions[4] = 1.53;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.46;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(6.0); //had to modify this line Trajectory message contains waypoints that are not strictly increasing in time.
 
}
//////////////
// Generates a simple trajectory with two waypoints to move TIAGo's arm towards Green bin
void waypoints_arm_postgraspGreen(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(2);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.07;
  goal.trajectory.points[index].positions[1] = 0.10;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = 0.93;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
 
  // Second trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.07;
  goal.trajectory.points[index].positions[1] = 0.10;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = -0.11;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(5.0);

 

}
// Generates a simple trajectory with two waypoints to move TIAGo's arm towards Blue bin
void waypoints_arm_postgraspBlue(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(3);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.80;
  goal.trajectory.points[index].positions[1] = 0.10;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = 0.93;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);


// Second trajectory point
  // Positions
   index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 1.25;
  goal.trajectory.points[index].positions[1] = 0.10;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = 0.93;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);


  // Third trajectory point
  // Positions
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 2.05;
  goal.trajectory.points[index].positions[1] = 0.10;
  goal.trajectory.points[index].positions[2] = -1.65;
  goal.trajectory.points[index].positions[3] = 0.94;
  goal.trajectory.points[index].positions[4] = 1.53;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 4 seconds after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(7.0);

 

}
//////////////
// Generates a simple trajectory with two waypoints to move TIAGo's arm towards neutral position
void waypoints_arm_postDropNeutralBlue(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(2);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 1.25;
  goal.trajectory.points[index].positions[1] = 0.20;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = 0.93;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.1;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);


// Second trajectory point
  // Positions
   index += 1;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.20;
  goal.trajectory.points[index].positions[1] = 0.20;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = 0.93;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(4.0);

}
// Generates a simple trajectory with two waypoints to move TIAGo's arm towards neutral position
void waypoints_arm_postDropNeutralGreen(control_msgs::FollowJointTrajectoryGoal& goal)
{
    // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("arm_1_joint");
  goal.trajectory.joint_names.push_back("arm_2_joint");
  goal.trajectory.joint_names.push_back("arm_3_joint");
  goal.trajectory.joint_names.push_back("arm_4_joint");
  goal.trajectory.joint_names.push_back("arm_5_joint");
  goal.trajectory.joint_names.push_back("arm_6_joint");
  goal.trajectory.joint_names.push_back("arm_7_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = 0.20;
  goal.trajectory.points[index].positions[1] = 0.20;
  goal.trajectory.points[index].positions[2] = -1.63;
  goal.trajectory.points[index].positions[3] = 0.93;
  goal.trajectory.points[index].positions[4] = 1.54;
  goal.trajectory.points[index].positions[5] = -1.36;
  goal.trajectory.points[index].positions[6] = 1.66;
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.5;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);


}
//////////////
// Generates a simple trajectory with one waypoint to move TIAGo's torso 
void waypoints_torso_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("torso_lift_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(1);
  goal.trajectory.points[index].positions[0] = 0.16;

  // Velocities
  goal.trajectory.points[index].velocities.resize(1);
  goal.trajectory.points[index].velocities[0] = 1.0;

  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);

}


// Generates a simple trajectory with one waypoint to move TIAGo's gripper 
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
////////////
//////////////
// Generates a simple trajectory with one waypoint to move TIAGo's gripper 
void waypoints_openGripper_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // The joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
  goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

  // Two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // Positions
  int index = 0;
  goal.trajectory.points[index].positions.resize(2);
  goal.trajectory.points[index].positions[0] = 0.09;
  goal.trajectory.points[index].positions[1] = 0.09;

  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
  ROS_INFO("opening gripper");
 
}
////////////

void moveTorso()
{
//Create an torso controller action client to move the TIAGo's torso
  torso_control_client_Ptr TorsoClient;
  createTorsoClient(TorsoClient);

  // Generates the goal for the TIAGo's torso
  control_msgs::FollowJointTrajectoryGoal torso_goal;
  waypoints_torso_goal(torso_goal);

  // Sends the command to start the given trajectory 1s from now
  torso_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  TorsoClient->sendGoal(torso_goal);
  ROS_INFO("moving the torso up...");
  // Wait for trajectory execution
  while(!(TorsoClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

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
  ROS_INFO("closing the gripper ...");
  // Wait for trajectory execution
  ros::Duration(4).sleep(); // sleep for QUATTRO seconds X VEDERE SE NON SI BLOCCA
  /* while(!(GripperClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }*/

}

void openGripper()
{
//Create an torso controller action client to move the TIAGo's gripper
  gripper_control_client_Ptr GripperClient;
  createGripperClient(GripperClient);

  // Generates the goal for the TIAGo's torso
  control_msgs::FollowJointTrajectoryGoal gripper_goal;
  waypoints_openGripper_goal(gripper_goal);

  // Sends the command to start the given trajectory 1s from now
  gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  GripperClient->sendGoal(gripper_goal);
  ROS_INFO("opening the gripper ...");
  // Wait for trajectory execution
  while(!(GripperClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

}
void PregraspMoveArm()
{
// Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  waypoints_arm_pregrasp(arm_goal);

  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);
  ROS_INFO("moving the arm towards the object...");

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

}
void PostgraspMoveArmGreen()
{
// Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal;
  waypoints_arm_postgraspGreen(arm_goal);

  // Sends the command to start the given trajectory 1s from now
  arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal);
  ROS_INFO("Moving towards the green bin...");

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

}
void PostgraspMoveArmBlue()
{
// Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal_blue;
  waypoints_arm_postgraspBlue(arm_goal_blue);

  // Sends the command to start the given trajectory 1s from now
  arm_goal_blue.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal_blue);
  ROS_INFO("Moving towards the blue bin...");

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

}

//////////
void PostDropNeutralBlueMove()
{
// Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal_neutralBlue;
  waypoints_arm_postDropNeutralBlue(arm_goal_neutralBlue);

  // Sends the command to start the given trajectory 1s from now
  arm_goal_neutralBlue.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal_neutralBlue);
  ROS_INFO("moving to sarting  position...");

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

}
////
//////////
void PostDropNeutralGreenMove()
{
// Create an arm controller action client to move the TIAGo's arm
  arm_control_client_Ptr ArmClient;
  createArmClient(ArmClient);

  // Generates the goal for the TIAGo's arm
  control_msgs::FollowJointTrajectoryGoal arm_goal_neutralGreen;
  waypoints_arm_postDropNeutralGreen(arm_goal_neutralGreen);

  // Sends the command to start the given trajectory 1s from now
  arm_goal_neutralGreen.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ArmClient->sendGoal(arm_goal_neutralGreen);
  ROS_INFO("moving to sarting position...");

  // Wait for trajectory execution
  while(!(ArmClient->getState().isDone()) && ros::ok())
  {
    ros::Duration(2).sleep(); // sleep for two seconds
  }

}
////
int flag = 1;//flag to sequence tiago`s activities
bool moveTorsoOnce = 1; //flag to make torso movement only once

void conveyorCallback(const std_msgs::Bool::ConstPtr & msg)
{
	flag = 3; //change value of global flag in order to enter in the successive step of motion of the switch statement of the main
}

ros::ServiceClient SpawnerClient;
sofar_assignment::Spawner spawner;

ros::ServiceClient ConveyorClient;
sofar_assignment::Conveyor conveyor;

ros::Subscriber ConveyorEndSub;

ros::ServiceClient ColorClient;
sofar_assignment::DetectColor detectColor;

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "run_traj_control");

  ROS_INFO("Starting simulation ...");
 
  // Precondition: Valid clock
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }

 SpawnerClient = nh.serviceClient<sofar_assignment::Spawner>("/spawn_box_service");
 ConveyorClient = nh.serviceClient<sofar_assignment::Conveyor>("/box_tracker_service");
 ConveyorEndSub = nh.subscribe("our_topic",1,conveyorCallback);

 ColorClient = nh.serviceClient<sofar_assignment::DetectColor>("/color_detect_service");
 //ros::Rate r(1000);

  while(ros::ok())
	{

	if(moveTorsoOnce)
	{
	  moveTorso();
          moveTorsoOnce=0;
	}

	switch (flag) {
	case 1:
	SpawnerClient.waitForExistence();
	SpawnerClient.call(spawner);
 	ROS_INFO("spawning box");
	if(spawner.response.spawned) flag = 2;
	else ROS_INFO("ERROR occured while spawning box");
	break;

	case 2:
	ConveyorClient.waitForExistence();
	ConveyorClient.call(conveyor);
 	ROS_INFO("moving conveyor");
	flag = 0;

		break;
	case 3:
	ColorClient.waitForExistence();
	ColorClient.call(detectColor);
	ROS_INFO("Detecting color");
	PregraspMoveArm();
	closeGripper();
	if(detectColor.response.color) //true == Green
	{
	  ROS_INFO("GREEN box detected");
	  PostgraspMoveArmGreen();
          openGripper();
	  PostDropNeutralGreenMove();
	
	}

	else			      //false == Blue
	 {
	   ROS_INFO("BLUE blue detected");
	   PostgraspMoveArmBlue();
           openGripper();
	   PostDropNeutralBlueMove();
	 } 
	 flag = 1;
	
		break;

	default:
	ros::spinOnce();
		break;
	ros::spinOnce();
	}      
 }

  
  return EXIT_SUCCESS;

}
















