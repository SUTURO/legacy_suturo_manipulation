/**
*/
#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_baseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_baseAction> Client;

using namespace std;


int main(int argc, char** argv)
{
	
	if (argc < 5)
	{
		ROS_INFO("arguments: position.x position.y position.z 0/1 ");
		return 1;
	}
	// create goal
	suturo_manipulation_msgs::suturo_manipulation_baseGoal goal;
	
	ros::init(argc, argv, "test_move_base_client");
	Client client("suturo_man_move_base_server", true);
	// wait for complete client initialisation
	ros::WallDuration(0.5).sleep();
	// waiting for connection
	client.waitForServer();
	ROS_INFO("Connected to server, ready to move base!");
			
	// set goal data
	goal.ps.pose.position.x = atof(argv[1]);
	ROS_INFO("set x done!");
	goal.ps.pose.position.y = atof(argv[2]);
	ROS_INFO("set y done!");
	goal.ps.pose.position.z = atof(argv[3]);
	ROS_INFO("set z done!");
	goal.ps.pose.orientation.x = 0;
	goal.ps.pose.orientation.y = 0;
	goal.ps.pose.orientation.z = 0;
	goal.ps.pose.orientation.w = 1;
	// if (argc == 5) goal.ps.header.frame_id = argv[4];
	if (atof(argv[4]) == 0){
		goal.ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);		
	} else {
	    goal.ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
	}
	// targetPose_.header.frame_id = "/map";
	goal.ps.header.frame_id = "/map";
	ROS_INFO("set frame done!");

	// send goal
	client.sendGoal(goal);

	client.waitForResult(ros::Duration(20.0));
	suturo_manipulation_msgs::suturo_manipulation_baseResultConstPtr r = client.getResult();
	
	// Get feedback from server
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
			ROS_INFO("moved base! result: %i", r->succ.type);
		} else {
			ROS_INFO("cant move base! result: %i", r->succ.type);
		}
		
	}
	printf("Current State: %s\n", client.getState().toString().c_str());
	return 0;
}
