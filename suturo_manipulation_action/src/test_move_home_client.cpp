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
		ROS_INFO("arguments: right_arm/left_arm/head");
		return 1;
	}

	ros::init(argc, argv, "test_move_home_client");
	Client client("suturo_man_move_home_server", true); 
	client.waitForServer();

	ROS_INFO("Connected to server, ready to go home!");
	
	suturo_manipulation_msgs::suturo_manipulation_homeGoal goal;
	
	goal.bodypart = argv[1];
	client.sendGoal(goal);
	
	//2x da es sonst nicht geht...
	client.waitForResult(ros::Duration(20.0));
	client.waitForResult(ros::Duration(20.0));
	suturo_manipulation_msgs::suturo_manipulation_homeResultConstPtr r = client.getResult();
	
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
			ROS_INFO("moved home :)!");
		} else {
			ROS_INFO("can't move home :(!");
		}
		
	} 
	return 0;
}
