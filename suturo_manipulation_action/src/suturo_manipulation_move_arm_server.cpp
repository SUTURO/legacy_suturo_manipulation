/**
* This class implements the action server to move an selected arm.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server_move;

tf::TransformListener* listener = NULL;

// Tranfsorm the incoming frame to /base_link
int kinectToOdom(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
    goalPose.pose.position.x = goalPoint.x;
    goalPose.pose.position.y = goalPoint.y;
    goalPose.pose.position.z = goalPoint.z;
    goalPose.pose.orientation.w = 1;
	
	// goal_frame
    const string odom = "/base_link";
    try{
		//transform pose from s to base_link and save it in pose again
		listener->transformPose(odom, goalPose, goalPose);
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
void execute(const suturo_manipulation_msgs::suturo_manipulation_moveGoalConstPtr& goal, Server_move* as)
{	
	// Set arm which should be moved
	string arm = goal->arm;
	// create a move result message
	suturo_manipulation_msgs::suturo_manipulation_moveResult r;	
	
	// Set header
    r.succ.header.stamp = ros::Time();
    // Set Answer fot planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	ROS_INFO("received arm: %s, x: %f, y: %f, z: %f", arm.c_str(), goal->ps.pose.position.x,
		goal->ps.pose.position.y, goal->ps.pose.position.z);
	
	//tranform pose
	geometry_msgs::PoseStamped odomPose;
	if (!kinectToOdom(odomPose, goal->ps.pose.position, goal->ps.header.frame_id.c_str())){
		// If tranfsormation fails, update the answer for planning to "FAIL" and set the server aborted
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		as->setAborted(r);
		return;
	}
	
	ROS_INFO("transformed to x: %f, y: %f, z: %f", odomPose.pose.position.x,
		odomPose.pose.position.y, odomPose.pose.position.z);	
	
	// set group to move
	move_group_interface::MoveGroup group(arm);

	// modifies the z-position, to touch object... dirty hack...
	odomPose.pose.position.z += 0.3;
	
	// set orientation to have a straight gripper
	odomPose.pose.orientation.x = 0;
	odomPose.pose.orientation.y = 0;
	odomPose.pose.orientation.z = 0;
	odomPose.pose.orientation.w = 1;
	
	// set Pose
	group.setPoseTarget(odomPose);
	ROS_INFO("current Position: x=%f, y=%f, z=%f", group.getCurrentPose().pose.position.x,
			group.getCurrentPose().pose.position.y,
			group.getCurrentPose().pose.position.z);
		
	//move arm
	if (group.move()){
	    r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    as->setSucceeded(r);
	} else {
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		as->setAborted(r);
	}	
	ROS_INFO("moved: %i", r.succ.type);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_move_arm_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);

	// create the action server
	Server_move server(n, "suturo_man_move_arm_server", boost::bind(&execute, _1, &server), false);
	// start the server
	server.start();
	
	ROS_INFO("Ready to move the arms!.");
	ros::spin();
	return 0;
}
