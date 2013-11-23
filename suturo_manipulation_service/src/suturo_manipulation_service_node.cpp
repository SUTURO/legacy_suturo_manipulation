#include "ros/ros.h"
#include <stdio.h>
#include "suturo_manipulation_msgs/suturo_manipulation_srv.h"
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include "tf/transform_listener.h"

using namespace std;
using namespace suturo_manipulation_msgs;

tf::TransformListener* listener = NULL;
geometry_msgs::PoseStamped kinectPose;
geometry_msgs::PoseStamped odomPose;

int kinectToOdom(double &x, double &y, double &z, const char* s)
{ 
	kinectPose.header.frame_id = s;
    kinectPose.pose.position.x = x;
    kinectPose.pose.position.y = y;
    kinectPose.pose.position.z = z;
    kinectPose.pose.orientation.x = 1;

    const string odom = "/odom_combined";
    listener->transformPose(odom, kinectPose, odomPose);

    return 0;
    }

bool moveb(suturo_manipulation_srv::Request  &req,
         suturo_manipulation_srv::Response &res)
{
    ros::AsyncSpinner spinner(1);
	spinner.start();

	suturo_manipulation_msgs::ActionAnswer answ;
	answ.header.stamp;
	answ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	//kinectToOdom(req.x, req.y, req.z);
	
	move_group_interface::MoveGroup group("left_arm");
	
	if (req.x == 0 && req.y == 0 && req.z == 0){
		group.setNamedTarget(req.arm+"_home");
	} else {
	/*	
		cout << group.getCurrentPose().pose.position.x << endl;
		cout << group.getCurrentPose().pose.position.y << endl;
		cout << group.getCurrentPose().pose.position.z << endl;
		group.setPositionTarget(req.x, req.y, req.z);	
    */
	}

    cout << "Positions- und Frame-Input:" << endl;
    cout << req.x <<" "<< req.y <<" "<< req.z <<" "<< req.arm << endl;
//    char frame = req.arm.c_str();
    kinectToOdom(req.x, req.y, req.z, req.arm.c_str());
    cout << odomPose.pose.position.x <<" "<< odomPose.pose.position.y <<" "<< odomPose.pose.position.z << odomPose.header.frame_id << endl;



	/*
	res.succ = group.move();
	if (res.succ){
		//ROS_INFO("%s moved to x=%f y=%f z=%f", req.arm.c_str(), req.x, req.y, req.z);
	} else {
                answ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		//ROS_INFO("Error while moving %s to x=%f y=%f z=%f", req.arm.c_str(), req.x, req.y, req.z);
	} */
	return res.succ != 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suturo_manipulation_srv", ros::init_options::AnonymousName);
  listener = new (tf::TransformListener);
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("suturo_manipulation_srv", moveb);
  ROS_INFO("Ready to moveit!.");
  ros::spin();

  return 0;
}
