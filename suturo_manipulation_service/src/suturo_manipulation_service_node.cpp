#include "ros/ros.h"
#include <stdio.h>
#include "suturo_manipulation_msgs/suturo_manipulation_srv.h"
#include <moveit/move_group_interface/move_group.h>

using namespace std;
using namespace suturo_manipulation_msgs;

int kinectToOdom(double &x, double &y, double &z)
{
	
	return 0;
}

bool moveb(suturo_manipulation_srv::Request  &req,
         suturo_manipulation_srv::Response &res)
{
    ros::AsyncSpinner spinner(1);
	spinner.start();
	
	//kinectToOdom(req.x, req.y, req.z);
	
	move_group_interface::MoveGroup group(req.arm);
	
	/*if (req.x == 0 && req.y == 0 && req.z == 0){
		group.setNamedTarget(req.arm+"_home");
	} else {
		
		cout << group.getCurrentPose().pose.position.x << endl;
		cout << group.getCurrentPose().pose.position.y << endl;
		cout << group.getCurrentPose().pose.position.z << endl;
		
		group.setPositionTarget(req.x, req.y, req.z);	
	}*/

	
	res.succ = group.move();
	if (res.succ){
		//ROS_INFO("%s moved to x=%f y=%f z=%f", req.arm.c_str(), req.x, req.y, req.z);
	} else {
		//ROS_INFO("Error while moving %s to x=%f y=%f z=%f", req.arm.c_str(), req.x, req.y, req.z);
	}
	return res.succ != 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suturo_manipulation_srv", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("suturo_manipulation_srv", moveb);
  ROS_INFO("Ready to moveit!.");
  ros::spin();

  return 0;
}
