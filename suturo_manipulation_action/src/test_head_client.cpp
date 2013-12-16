#include <ros/ros.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <actionlib/client/simple_action_client.h>
#include "suturo_perception_msgs/PrologFinish.h"
#include "suturo_perception_msgs/GetClusters.h"
#include "suturo_perception_msgs/PrologQuery.h"
#include "suturo_perception_msgs/PrologNextSolution.h"

typedef actionlib::SimpleActionClient<suturo_manipulation_msgs::suturo_manipulation_headAction> Head_client;

using namespace std;

int main(int argc, char** argv)
{
	
	// if (argc != 6)
	// {
	// 	ROS_INFO("usage: arm X Y Z start_frame_id");
	// 	return 1;
	// }

	// Dummy Percived Object
	suturo_perception_msgs::PerceivedObject *p_goal = new suturo_perception_msgs::PerceivedObject();
	p_goal->c_id = 1;
	p_goal->c_shape = 2;
	p_goal->c_volume = 0.0005;
	p_goal->c_centroid.x = -0.378;
	p_goal->c_centroid.y = 0.187;
	p_goal->c_centroid.z = 0.991;
	p_goal->frame_id = "/head_mount_kinect_rgb_optical_frame";
	// p_goal->c_color_average_r = it->c_color_average_r;
	// p_goal->c_color_average_g = it->c_color_average_g;
	// p_goal->c_color_average_b = it->c_color_average_b;
	// these are not set for now
	p_goal->recognition_label_2d = "";

	// Dummy Goal
	suturo_manipulation_msgs::suturo_manipulation_headGoal goal;

	goal.p.c_centroid.x = p_goal->c_centroid.x;
	goal.p.c_centroid.y = p_goal->c_centroid.y;
	goal.p.c_centroid.z = p_goal->c_centroid.z;
	goal.p.c_id = p_goal->c_id;
	goal.p.c_shape = p_goal->c_shape;
	goal.p.c_volume = p_goal->c_volume;
	goal.p.frame_id = p_goal->frame_id;

	ros::init(argc, argv, "test_head_client");
	Head_client client("move_head_server", true); 
	client.waitForServer();
	client.sendGoal(goal);
		
	// 	suturo_manipulation_msgs::suturo_manipulation_headGoal goal;
	// 	//goal.p.frame_id = frame;
		
	// 	goal.arm = argv[1];
	// 	client.sendGoal(goal);
		
		
	// 	//2x da es sonst nicht geht...
	// 	client.waitForResult(ros::Duration(20.0));
	// 	client.waitForResult(ros::Duration(20.0));
	// 	suturo_manipulation_msgs::suturo_manipulation_headResultConstPtr r = client.getResult();
		
	// 	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	// 		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
	// 			ROS_INFO("moved!!!!!!! result: %i", r->succ.type);
	// 		}else{
	// 			ROS_INFO("not moved!!!!!!! result: %i", r->succ.type);
	// 		}
			
	// 	}
	// 	printf("Current State: %s\n", client.getState().toString().c_str());
	// } else {
	
	// 	suturo_manipulation_msgs::suturo_manipulation_moveGoal goal;
	// 	goal.p.frame_id = frame;
		
	// 	ros::init(argc, argv, "test_action_client");
	// 	Client client("move_action_server", true); // true -> don't need ros::spin()
	// 	client.waitForServer();
		
	// 	goal.arm = argv[1];
	// 	goal.p.c_centroid.x = atof(argv[2]);
	// 	goal.p.c_centroid.y = atof(argv[3]);
	// 	goal.p.c_centroid.z = atof(argv[4]);
	// 	client.sendGoal(goal);
		
		
	// 	//2x da es sonst nicht geht...
	// 	client.waitForResult(ros::Duration(20.0));
	// 	client.waitForResult(ros::Duration(20.0));
	// 	suturo_manipulation_msgs::suturo_manipulation_moveResultConstPtr r = client.getResult();
		
	// 	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	// 		if (suturo_manipulation_msgs::ActionAnswer::SUCCESS == r->succ.type){
	// 			ROS_INFO("moved!!!!!!! result: %i", r->succ.type);
	// 		}else{
	// 			ROS_INFO("not moved!!!!!!! result: %i", r->succ.type);
	// 		}
			
	// 	}
	// 	printf("Current State: %s\n", client.getState().toString().c_str());
	// }
	// return 0;
}
