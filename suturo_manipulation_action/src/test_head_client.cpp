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
	if (argc != 5)
	{
		ROS_INFO("usage: X Y Z frame_id");
		return 1;
	}
	
	// initiliaze the client
	ros::init(argc, argv, "test_head_client");
	ros::start();
	
	// Dummy PosedStamped Object
	geometry_msgs::PoseStamped ps;

	// Set PoseStamped Object with input
	ps.pose.position.x = atof(argv[1]);
	ps.pose.position.y = atof(argv[2]);
	ps.pose.position.z = atof(argv[3]);
	ps.header.frame_id = argv[4];
	ps.header.stamp = ros::Time::now();

	ROS_INFO("PosedStamped Object done!");

	// Dummy Goal
	suturo_manipulation_msgs::suturo_manipulation_headGoal goal;

	// Set goal with PoseStamped Object data
	goal.ps.pose.position.x = ps.pose.position.x;
	goal.ps.pose.position.y = ps.pose.position.y;
	goal.ps.pose.position.z = ps.pose.position.z;
	goal.ps.header.frame_id = ps.header.frame_id;

	ROS_INFO("HeadGoal done!");

	// create client and connect to server
	Head_client client("suturo_man_move_head_server", true); 
	// waiting for connection
	client.waitForServer();
	ROS_INFO("connected! let's move the head!");
	// send the goal
	client.sendGoal(goal);
	ROS_INFO("goal sended!");
}