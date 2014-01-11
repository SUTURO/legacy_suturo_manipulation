/**
* This class implements the action server for moving the head.
* If the action server is called, the server publishes 
* the goal to a topic. The topic is
* /suturo/head_controller_goal_point .
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_headAction> Server_head;

tf::TransformListener* listener = NULL;

// Tranfsorm the incoming frame to /head_mount_kinect_rgb_optical_frame for the head move controller
int tranform(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
    goalPose.pose.position.x = goalPoint.x;
    goalPose.pose.position.y = goalPoint.y;
    goalPose.pose.position.z = goalPoint.z;
    goalPose.pose.orientation.w = 1;
	
	// goal_frame
    const string goal_frame = "/head_mount_kinect_rgb_optical_frame";
    // const string goal_frame = "/base_link";

	ROS_INFO("Beginn der Transformation");
    try{
		//transform pose from s to odom_combined and save it in pose again
		listener->transformPose(goal_frame, goalPose, goalPose);
	}catch(...){
		ROS_INFO("ERROR: Transformation failed.");
		return 0;
	}

    return 1;
}

/**
* This method starts the transformation to the right frame and 
* publishes the tranformed goal.
*/
void execute(const suturo_manipulation_msgs::suturo_manipulation_headGoalConstPtr& goal, ros::Publisher* publisher, Server_head* head_server)
{	
	suturo_manipulation_msgs::suturo_manipulation_headResult r;	
	
	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer fot planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	//tranform pose
	geometry_msgs::PoseStamped odomPose;
	if (!tranform(odomPose, goal->ps.pose.position, goal->ps.header.frame_id.c_str())){
		// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		head_server->setAborted(r);
		return;
	}

	// Publish goal on topic /suturo/head_controller
	if( !publisher ) {
  		ROS_WARN("Publisher invalid!");
  		head_server->setAborted(r);
	} else {
		ROS_INFO("Published goal: x: %f, y: %f, z: %f in Frame %s", odomPose.pose.position.x,
		odomPose.pose.position.y, odomPose.pose.position.z, odomPose.header.frame_id.c_str());	
		publisher->publish(odomPose);
		ROS_INFO("Goal published!");
		head_server->setSucceeded(r);
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_head_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);
	
	// Publish a topic for the head controller
	ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);

	// create the action server
	Server_head head_server(n, "suturo_man_move_head_server", boost::bind(&execute, _1, &head_publisher, &head_server), false);
	// start the server
	head_server.start();

	ROS_INFO("Ready to move the head!");
	ros::spin();
	return 0;
}
