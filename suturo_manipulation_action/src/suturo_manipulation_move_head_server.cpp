/**
*
*
*
*
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_headAction> Server_head;

tf::TransformListener* listener = NULL;

// Tranfsorm the incoming frame to ??? for the head move controller
int tranform(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
    goalPose.pose.position.x = goalPoint.x;
    goalPose.pose.position.y = goalPoint.y;
    goalPose.pose.position.z = goalPoint.z;
    goalPose.pose.orientation.w = 1;
	
    const string goal_frame = "/base_link";
	// ROS_INFO("Beginn der Transformation von %s zu " + goal_frame, s);
    try{
		//transform pose from s to odom_combined and save it in pose again
		listener->transformPose(goal_frame, goalPose, goalPose);
	}catch(...){
		ROS_INFO("ERROR: Transformation failed.");
		return 0;
	}

    return 1;
}


void execute(const suturo_manipulation_msgs::suturo_manipulation_headGoalConstPtr& goal, Server_head* head_server)
{	
	suturo_manipulation_msgs::suturo_manipulation_headResult r;	
	
	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer fot planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	//tranform pose
	geometry_msgs::PoseStamped odomPose;
	if (!tranform(odomPose, goal->p.c_centroid, goal->p.frame_id.c_str())){
		// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		head_server->setAborted(r);
		return;
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_head_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);
	
	// create the action server
	Server_head head_server(n, "move_head_server", boost::bind(&execute, _1, &head_server), false);
	// start the server
	head_server.start();

	// Publish a topic for the head controller
	// ros::Publisher head_publisher = n.advertise<suturo_manipulation_msgs::suturo_manipulation_headAction>("/suturo/head_controller", 1000);
	// ros::Publisher head_publisher = n.advertise<std_msgs::String>("/suturo/head_controller", 1000);
	
	ROS_INFO("Ready to move the head!");
	ros::spin();
	return 0;
}
