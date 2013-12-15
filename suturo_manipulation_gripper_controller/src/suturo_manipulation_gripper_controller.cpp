#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include "suturo_manipulation_gripper_controller.h"

// Our Action interface type, provided as a typedef for convenience

  //Action client initialization
Gripper::Gripper()
{

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
	r_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
	l_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!r_gripper_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
    
    //wait for the gripper action server to come up 
    while(!l_gripper_client_->waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
    }
}

Gripper::~Gripper()
{
    delete r_gripper_client_;
    delete l_gripper_client_;
}

//Open the gripper
actionlib::SimpleClientGoalState open_gripper(GripperClient* gripper_client_)
{
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.085;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else 
      ROS_INFO_STREAM("The gripper failed to open! :(");
    return gripper_client_->getState();
}

actionlib::SimpleClientGoalState Gripper::open_r_gripper()
{
	return open_gripper(r_gripper_client_);
}

actionlib::SimpleClientGoalState Gripper::open_l_gripper()
{
	return open_gripper(l_gripper_client_);
}

//Close the gripper
actionlib::SimpleClientGoalState close_gripper(GripperClient* gripper_client_)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("The gripper closed!");
    else
		ROS_INFO("The gripper failed to close.");
	return gripper_client_->getState();
}

actionlib::SimpleClientGoalState Gripper::close_r_gripper()
{
	return close_gripper(r_gripper_client_);
}

actionlib::SimpleClientGoalState Gripper::close_l_gripper()
{
	return close_gripper(l_gripper_client_);
}
