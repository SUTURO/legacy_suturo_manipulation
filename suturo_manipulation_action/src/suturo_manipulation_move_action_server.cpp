#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server;

void execute(const suturo_manipulation_msgs::suturo_manipulation_moveGoalConstPtr& goal, Server* as)
{	
	suturo_manipulation_msgs::suturo_manipulation_moveResult r;	
	double x = goal->p.c_centroid.x;
	double y = goal->p.c_centroid.y;
	double z = goal->p.c_centroid.z;
	string arm = goal->arm;
	
	ROS_INFO("received arm: %s, x: %f, y: %f, z: %f", arm.c_str(), x, y, z);
	//kinectToOdom(req.x, req.y, req.z);
	ROS_INFO("transformed to x: %f, y: %f, z: %f", x, y, z);
	move_group_interface::MoveGroup group(arm);
	
	r.succ.header.stamp;
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	ROS_INFO("link: %s", group.getEndEffectorLink().c_str());
	if (x == 0 && y == 0 && z == 0){
		group.setNamedTarget(arm+"_home");
		ROS_INFO("current pos: x=%f, y=%f, z=%f", group.getCurrentPose().pose.position.x,
				group.getCurrentPose().pose.position.y,
				group.getCurrentPose().pose.position.z);
	} else {
		ROS_INFO("current pos: x=%f, y=%f, z=%f", group.getCurrentPose().pose.position.x,
				group.getCurrentPose().pose.position.y,
				group.getCurrentPose().pose.position.z);
		group.setPositionTarget(x, y, z);	
	}
	
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
  Server server(n, "move_action_server", boost::bind(&execute, _1, &server), false);
  server.start();
  ROS_INFO("Ready to moveit!.");
  ros::spin();
  return 0;
}
