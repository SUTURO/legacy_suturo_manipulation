/**
* This clas implements a dummy client for our head action server.
* The client sends a goal to the server, which have to publish this data.
*/
#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_headAction> Head_client;

using namespace std;


int main(int argc, char** argv)
{
	// Dummy PosedStamped Object
	geometry_msgs::PoseStamped ps;

	ps.pose.position.x = -0.378;
	ps.pose.position.y = 0.187;
	ps.pose.position.z = 0.991;
	ps.header.frame_id = "/head_mount_kinect_rgb_optical_frame";

	ROS_INFO("PosedStamped Object done!");

	// Dummy Goal
	suturo_manipulation_msgs::suturo_manipulation_headGoal goal;

	goal.ps.pose.position.x = ps.pose.position.x;
	goal.ps.pose.position.y = ps.pose.position.y;
	goal.ps.pose.position.z = ps.pose.position.z;
	goal.ps.header.frame_id = ps.header.frame_id;

	ROS_INFO("HeadGoal done!");

	// initiliaze the client
	ros::init(argc, argv, "test_head_client");
	Head_client client("suturo_man_move_head_server", true); 
	// waiting for connection
	client.waitForServer();
	ROS_INFO("connected!");
	// send the goal
	client.sendGoal(goal);
	ROS_INFO("goal sended!");
}
