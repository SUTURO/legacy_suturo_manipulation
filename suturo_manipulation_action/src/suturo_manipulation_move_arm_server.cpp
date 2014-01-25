/**
* This class implements the action server to move an selected arm.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server;

tf::TransformListener* listener = NULL;

// Transform the incoming frame to /base_link
int transform(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
    goalPose.pose.position.x = goalPoint.x;
    goalPose.pose.position.y = goalPoint.y;
    goalPose.pose.position.z = goalPoint.z;
    goalPose.pose.orientation.w = 1;
	
	// goal_frame
    const string goalFrame = "/base_link";

	ROS_INFO("Beginn der Transformation von %s zu /base_link", s);
    try{
		//transform pose from s to base_link and save it in pose again
		listener->transformPose(goalFrame, goalPose, goalPose);
	}catch(...){
		ROS_INFO("ERROR: Transformation failed.");
		return 0;
	}

    return 1;
}

/**
* This method starts the transformation to the right frame and 
* calls move() to move the selected arm.
*/
void moveArm(const suturo_manipulation_msgs::suturo_manipulation_moveGoalConstPtr& goal, Server* server_arm)
{	

	// create a move result message
	suturo_manipulation_msgs::suturo_manipulation_moveResult r;	
	
	// Set header
    r.succ.header.stamp = ros::Time();
    // Set Answer fot planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

	// Set arm which should be moved
	string arm = goal->bodypart.bodyPart;
	if (arm != suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM && arm != suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){
		ROS_INFO("Unknown arm! Please use suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM or suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM as names!\n");
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_arm->setAborted(r);	
	}
	
	ROS_INFO("received arm: %s, x: %f, y: %f, z: %f", arm.c_str(), goal->ps.pose.position.x,
		goal->ps.pose.position.y, goal->ps.pose.position.z);
	
	//tranform pose
	geometry_msgs::PoseStamped transformedPose;
	if (!transform(transformedPose, goal->ps.pose.position, goal->ps.header.frame_id.c_str())){
		ROS_INFO("Transformation failed!\n");
		// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_arm->setAborted(r);
	}

	//Orientierung des End-Effektors wieder auf w=1 setzen
	transformedPose.pose.orientation.x = 0;
	transformedPose.pose.orientation.y = 0;
	transformedPose.pose.orientation.z = 0;
	transformedPose.pose.orientation.w = 1;
	ROS_INFO("transformed to x: %f, y: %f, z: %f in Frame %s", transformedPose.pose.position.x,
		transformedPose.pose.position.y, transformedPose.pose.position.z, transformedPose.header.frame_id.c_str());	
	
	// set group to move
	move_group_interface::MoveGroup group(arm);
	
	// set orientation to have a straight gripper
	transformedPose.pose.orientation.x = 0;
	transformedPose.pose.orientation.y = 0;
	transformedPose.pose.orientation.z = 0;
	transformedPose.pose.orientation.w = 1;
	
	// set Pose
	group.setPoseTarget(transformedPose);
	ROS_INFO("current Position: x=%f, y=%f, z=%f in Frame %s", group.getCurrentPose().pose.position.x,
			group.getCurrentPose().pose.position.y,
			group.getCurrentPose().pose.position.z, group.getCurrentPose().header.frame_id.c_str());
		
	//move arm
	if (group.move()){
		ROS_INFO("Arm moved!\n");
	    r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    server_arm->setSucceeded(r);
	} else {
		ROS_INFO("Moving failed!\n");
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_arm->setAborted(r);
	}	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_move_arm_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);

	// create the action server
	Server server_arm(n, "suturo_man_move_arm_server", boost::bind(&moveArm, _1, &server_arm), false);
	// start the server
	server_arm.start();
	
	ROS_INFO("Ready to move the arms!.");
	ros::spin();
	return 0;
}
