/**
* This clas implements a dummy client for our move arm server.
* The client sends a goal to the server, which moves the arm to 
* the given position.
*/
#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_moveAction> Client;

using namespace std;


int main(int argc, char** argv)
{
	
	if (argc != 9)
	{
		ROS_INFO("arguments: x y z raw pitch yaw frame arm");
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
	goal.ps.pose.position.y = atof(argv[2]);
	goal.ps.pose.position.z = atof(argv[3]);
	
	goal.ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[4]), atof(argv[5]), atof(argv[6]));;

	goal.ps.header.frame_id = argv[7];
	goal.bodypart.bodyPart = argv[8];

	ROS_INFO_STREAM("Move to: " << goal.ps);

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
