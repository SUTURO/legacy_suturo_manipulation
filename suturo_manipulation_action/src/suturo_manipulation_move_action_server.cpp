#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server;

void execute(const suturo_manipulation_msgs::suturo_manipulation_moveGoalConstPtr& goal, Server* as)
{	
	double x = goal->p.c_centroid.x;
	double y = goal->p.c_centroid.y;
	double z = goal->p.c_centroid.z;
	string arm = goal->arm;
	
	ROS_INFO("received arm: %s, x: %f, y: %f, z: %f", arm.c_str(), x, y, z);
	//kinectToOdom(req.x, req.y, req.z);
	ROS_INFO("transformed to x: %f, y: %f, z: %f", x, y, z);
	move_group_interface::MoveGroup group(arm);
	
	if (x == 0 && y == 0 && z == 0){
		group.setNamedTarget(arm+"_home");
	} else {
		cout << "current pos\n";
		cout << group.getCurrentPose().pose.position.x << endl;
		cout << group.getCurrentPose().pose.position.y << endl;
		cout << group.getCurrentPose().pose.position.z << endl;
		
		group.setPositionTarget(x, y, z);	
	}
	int res = group.move();
	ROS_INFO("moved: %i", res);
	
	as->setSucceeded();
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
