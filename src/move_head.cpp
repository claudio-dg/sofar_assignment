// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>


// client definition of action service
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;

typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr; // pointer to the client

PointHeadClientPtr pointHeadClient; // assign pointer

// Create a ROS action client to move TIAGo's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );
  ROS_INFO("Creating action cadaddad...");

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}

void moveHead(void){

    geometry_msgs::PointStamped pointStamped;
 
    pointStamped.header.frame_id = "/xtion_rgb_optical_frame";
    //pointStamped.header.stamp    = ros::Time;
 
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
    ROS_INFO("BNBNBN");
    ros::Duration(0.5).sleep();

}

// Entry point
int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "move_head");

  ROS_INFO("Starting move_head application ...");

  //ros::Duration(5).sleep();
 
  // Precondition: Valid clock (?????????)
  ros::NodeHandle nh;
  if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
  {
    ROS_FATAL("Timed-out waiting for valid time.");
    return EXIT_FAILURE;
  }
   
   // Create a point head action client to move the TIAGo's head
  createPointHeadClient( pointHeadClient );

    bool done = 0;
    
    if(done == 0){
        ROS_INFO("AAAAAAAAAAAAAA");
        moveHead();
        done = 1;
    }


 

  //enter a loop that processes ROS callbacks. Press CTRL+C to exit the loop
  ros::spin(); 

  return EXIT_SUCCESS;
}
