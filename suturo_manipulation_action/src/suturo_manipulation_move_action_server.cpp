#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <suturo_manipulation_msgs/suturo_manipulation_homeAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server_move;
typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_homeAction> Server_home;

tf::TransformListener* listener = NULL;

int kinectToOdom(geometry_msgs::PoseStamped &goalPose,
					geometry_msgs::Point goalPoint, const char* s)
{ 
	//save goal position in pose
	goalPose.header.frame_id = s;
    goalPose.pose.position.x = goalPoint.x;
    goalPose.pose.position.y = goalPoint.y;
    goalPose.pose.position.z = goalPoint.z;
    goalPose.pose.orientation.w = 1;
	
    const string odom = "/odom_combined";
    try{
		//transform pose from s to odom_combined and save it in pose again
		listener->transformPose(odom, goalPose, goalPose);
	}catch(...){
		ROS_INFO("ERROR: Transformation failed.");
		return 0;
	}

    return 1;
}


void execute(const suturo_manipulation_msgs::suturo_manipulation_moveGoalConstPtr& goal, Server_move* as)
{	
	string arm = goal->arm;
	suturo_manipulation_msgs::suturo_manipulation_moveResult r;	
	
	r.succ.header.stamp = ros::Time();
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	ROS_INFO("received arm: %s, x: %f, y: %f, z: %f", arm.c_str(), goal->p.c_centroid.x,
		goal->p.c_centroid.y, goal->p.c_centroid.z);
	
	//tranform pose
	geometry_msgs::PoseStamped odomPose;
	if (!kinectToOdom(odomPose, goal->p.c_centroid, goal->p.frame_id.c_str())){
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		as->setAborted(r);
		return;
	}
	
	ROS_INFO("transformed to x: %f, y: %f, z: %f", odomPose.pose.position.x,
		odomPose.pose.position.y, odomPose.pose.position.z);	
	
	//set goal Pose
	move_group_interface::MoveGroup group(arm);

	odomPose.pose.position.z += 0.3;
	
	odomPose.pose.orientation.x = 0;
	odomPose.pose.orientation.y = 0;
	odomPose.pose.orientation.z = 0;
	odomPose.pose.orientation.w = 1;
	
	
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

void home(const suturo_manipulation_msgs::suturo_manipulation_homeGoalConstPtr& goal, Server_home* as)
{	
	suturo_manipulation_msgs::suturo_manipulation_homeResult r;	
	string arm = goal->arm;
	
	move_group_interface::MoveGroup group(arm);
	
	r.succ.header.stamp = ros::Time();
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	group.setNamedTarget(arm+"_home");
	ROS_INFO("current pos: x=%f, y=%f, z=%f", group.getCurrentPose().pose.position.x,
			group.getCurrentPose().pose.position.y,
			group.getCurrentPose().pose.position.z);
	
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
	ros::init(argc, argv, "suturo_manipulation_move_server");
	ros::NodeHandle n;
	listener = new (tf::TransformListener);
	
	Server_move server(n, "move_action_server", boost::bind(&execute, _1, &server), false);
	Server_home server_home(n, "home_action_server", boost::bind(&home, _1, &server_home), false);
	server.start();
	server_home.start();
	
	ROS_INFO("Ready to moveit!.");
	ros::spin();
	return 0;
}
