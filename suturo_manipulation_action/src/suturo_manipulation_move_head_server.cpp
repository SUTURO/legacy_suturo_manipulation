/**
* This class implements the action server for moving the head.
* If the action server is called, the server publishes 
* the goal to the topic /suturo/head_controller_goal_point .
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_headAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <tf/transform_listener.h>
#include <suturo_manipulation_msgs/torque_values.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_headAction> Server;

// TF Listener...
tf::TransformListener* listener = NULL;

double yaw_error, pitch_error;

// Transform the incoming frame to /torso_lift_link for the head move controller
int transform(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
	goalPose.pose.position.x = goalPoint.x;
	goalPose.pose.position.y = goalPoint.y;
	goalPose.pose.position.z = goalPoint.z;
	goalPose.pose.orientation.x = 0;
	goalPose.pose.orientation.y = 0;
	goalPose.pose.orientation.z = 0;
	goalPose.pose.orientation.w = 1;
	
	// goal_frame
  const string goal_frame = "/torso_lift_link";

	ROS_INFO("Begin transformation");
  try{
		//transform pose from s to -torso_lift_link and save it in pose again
		listener->transformPose(goal_frame, goalPose, goalPose);
	}catch(...){
		ROS_INFO("ERROR: Transformation failed.");
		return 0;
	}

    return 1;
}

/**
* This method starts the transformation to the right frame and 
* publishes the transformed goal.
*/
void moveHead(const suturo_manipulation_msgs::suturo_manipulation_headGoalConstPtr& goal, 
			ros::Publisher* publisher, Server* server_head)
{	
	ROS_INFO("callback moveHead() begins...");
	suturo_manipulation_msgs::suturo_manipulation_headResult r;	

	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer fot planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	geometry_msgs::PoseStamped transformedPose;

	//transform pose
	if (!transform(transformedPose, goal->ps.pose.position, goal->ps.header.frame_id.c_str())){
		// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_head->setAborted(r);
	}

	// Publish goal on topic /suturo/head_controller_goal_point
	if( !publisher ) {
		ROS_WARN("Publisher invalid!\n");
	  	r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
	  	server_head->setAborted(r);
	} else {
		ROS_INFO("Published goal: x: %f, y: %f, z: %f in Frame %s", transformedPose.pose.position.x,
		transformedPose.pose.position.y, transformedPose.pose.position.z, transformedPose.header.frame_id.c_str());	
		publisher->publish(transformedPose);
		ROS_INFO("Goal published!\n");

		int counter = 0;
		int duration = 0;
		// Checking of the goal is reached
		while (duration < 3 && counter < 15){
			if (yaw_error < 5 && pitch_error < 1){
				duration++;
				ROS_INFO_STREAM("Duration Head Server: " <<  duration);
			} else {
				duration = 0;
				ROS_INFO("Over Treshold Head Server");
			}
			ros::Duration(1).sleep();
			counter++;
		}
		if(counter >= 15){
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_head->setAborted(r);
		} else {
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_head->setSucceeded(r);			
		}

	}
}

/**
* This callback saves the current controller_errors in class variables
*/
void subCallback(const suturo_manipulation_msgs::torque_valuesConstPtr& msg){
	yaw_error = msg->yaw_error;
	pitch_error = msg->pitch_error;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_head_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);
	
	// Publish a topic for the head controller
	ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);

	// Subscribe to the controller_errors
	ros::Subscriber sub = n.subscribe("/suturo/head_controller/torque_values", 1000, subCallback);

	// create the action server
	Server server_head(n, "suturo_man_move_head_server", boost::bind(&moveHead, _1, &head_publisher, &server_head), false);
	// start the server
	server_head.start();

	ROS_INFO("Ready to move the head!");
	ros::spin();
	return 0;
}
