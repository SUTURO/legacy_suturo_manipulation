/**
* This clas implements a dummy client for our move home server.
* The client sends a goal to the server, which moves the given
* bodypart in a home position.
*/
#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_homeAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_homeAction> Client;

using namespace std;


int main(int argc, char** argv)
{
	
	if (argc != 2)
	{
		ROS_INFO("arguments: right_arm/left_arm/head/both_arms");
		return 1;
	}

	ros::init(argc, argv, "test_move_home_client");
	// create client and connect to server
	Client client("suturo_man_move_home_server", true); 
	// wait for complete client initialisation
	ros::WallDuration(0.5).sleep();
	// waiting for connection
	client.waitForServer();
	ROS_INFO("Connected to server, ready to go home!");
	
	// Create goal
	suturo_manipulation_msgs::suturo_manipulation_homeGoal goal;
	
	// Set goal data
	goal.bodypart.bodyPart = argv[1];
	// send goal to server
	client.sendGoal(goal);
	
	client.waitForResult(ros::Duration(20.0));
	suturo_manipulation_msgs::suturo_manipulation_homeResultConstPtr r = client.getResult();
	
	// Get feedback from server
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
			ROS_INFO("moved home :)!");
		} else {
			ROS_INFO("can't move home :(!");
		}
	} 
	ROS_INFO_STREAM(r->succ.type);
	return 0;
}
