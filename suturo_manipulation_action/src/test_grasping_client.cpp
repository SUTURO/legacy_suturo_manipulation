/**
* This clas implements a dummy client for our grasping server.
* The client sends a goal (object) to the server, which have to grasp this object.
*/
#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Client;

using namespace std;


int main(int argc, char** argv)
{
	
	if (argc != 5)
	{
		ROS_INFO("arguments: box1/box2/beer1 newton right_arm/left_arm grasp(1)/drop(0)");
		return 1;
	}
		
	ros::init(argc, argv, "test_action_pick_client");
	// waiting for connection
	Client client("suturo_man_grasping_server", true);
	// wait for complete client initialisation
	ros::WallDuration(0.5).sleep();
	// waiting for connection
	client.waitForServer();
	ROS_INFO("Connected to server, ready to pick");

	// create goal and put data in it
	suturo_manipulation_msgs::suturo_manipulation_graspingGoal goal;
	goal.goal.header.seq = 0;
	goal.goal.header.stamp = ros::Time();
	goal.goal.header.frame_id = "/base_footprint";
	goal.goal.objectName = argv[1];
	// if(arm=="left_arm"){
	// 	goal.goal.bodypart.bodyPart = suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM;		
	// } else {
	// 	goal.goal.bodypart.bodyPart = suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM;
	// }
	goal.goal.bodypart.bodyPart = argv[3];
	goal.goal.newton = atof(argv[2]);
	goal.goal.grasp = atof(argv[4]);

	// send the goal to server
	client.sendGoal(goal);
	client.waitForResult(ros::Duration(20.0));
	
	
	suturo_manipulation_msgs::suturo_manipulation_graspingResultConstPtr r = client.getResult();
	// Get feedback from server
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
			ROS_INFO("object grasped! result: %i", r->succ.type);
		}else{
			ROS_INFO("object not grasped! result: %i", r->succ.type);
		}
		
	}
	return 0;
}
