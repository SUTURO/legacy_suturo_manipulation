/**
* This clas implements a dummy client for our head action server.
* The client sends a goal to the server, which have to publish this data.
*/
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

	ROS_INFO("PerceivedObject done!");

	// Dummy Goal
	suturo_manipulation_msgs::suturo_manipulation_headGoal goal;

	goal.p.c_centroid.x = p_goal->c_centroid.x;
	goal.p.c_centroid.y = p_goal->c_centroid.y;
	goal.p.c_centroid.z = p_goal->c_centroid.z;
	goal.p.c_id = p_goal->c_id;
	goal.p.c_shape = p_goal->c_shape;
	goal.p.c_volume = p_goal->c_volume;
	goal.p.frame_id = p_goal->frame_id;

	ROS_INFO("HeadGoal done!");

	// initiliaze the client
	ros::init(argc, argv, "test_head_client");
	Head_client client("move_head_server", true); 
	// waiting for connection
	client.waitForServer();
	ROS_INFO("connected!");
	// send the goal
	client.sendGoal(goal);
	ROS_INFO("goal sended!");
}
