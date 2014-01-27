/**
* This clas implements a dummy client for our move arm server.
* The client sends a goal to the server, which moves the arm to 
* the given position.
*/
#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_moveAction> Client;

using namespace std;


int main(int argc, char** argv)
{
	
	if (argc != 6)
	{
		ROS_INFO("arguments: x y z frame arm");
		return 1;
	}
	// create goal
	suturo_manipulation_msgs::suturo_manipulation_moveGoal goal;
	
	ros::init(argc, argv, "test_move_arm_client");
	Client client("suturo_man_move_arm_server", true);
	// wait for complete client initialisation
	ros::WallDuration(0.5).sleep();
	// waiting for connection
	client.waitForServer();
	ROS_INFO("Connected to server, ready to move arm!");
			
	// set goal data
	goal.ps.pose.position.x = atof(argv[1]);
	ROS_INFO("set x done!");
	goal.ps.pose.position.y = atof(argv[2]);
	ROS_INFO("set y done!");
	goal.ps.pose.position.z = atof(argv[3]);
	ROS_INFO("set z done!");
	goal.ps.header.frame_id = argv[4];;
	ROS_INFO("set frame done!");
	goal.bodypart.bodyPart = argv[5];
	ROS_INFO("set arm done!");

	// send goal
	client.sendGoal(goal);

	client.waitForResult(ros::Duration(20.0));
	suturo_manipulation_msgs::suturo_manipulation_moveResultConstPtr r = client.getResult();
	
	// Get feedback from server
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
			ROS_INFO("moved arm! result: %i", r->succ.type);
		} else {
			ROS_INFO("cant move arm! result: %i", r->succ.type);
		}
		
	}
	printf("Current State: %s\n", client.getState().toString().c_str());
	return 0;
}
