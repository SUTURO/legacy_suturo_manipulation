/**
* This clas implements a dummy subscriber for our head_controller_goal_point topic.
* The subscriber should be able to get the data, if something is published by
* the action server.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <tf/transform_listener.h>

/**
* If something was published, this method is called.
* This method shows information from the published data
* on the console.
*/
void chatterCallback(geometry_msgs::PoseStamped msg)
{
  ROS_INFO("I heard: x: %f, y: %f, z: %f in Frame %s", msg.pose.position.x,
		msg.pose.position.y, msg.pose.position.z, msg.header.frame_id.c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_test_head_subscriber");
	ros::NodeHandle n;
	// Subscribe the topic, call chatterCallback if data is published
	ros::Subscriber sub = n.subscribe("/suturo/head_controller_goal_point", 1000, chatterCallback);
	ROS_INFO("Subscribed!");
	ros::spin();
}