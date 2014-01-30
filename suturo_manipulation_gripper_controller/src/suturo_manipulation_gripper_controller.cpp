#include "suturo_manipulation_gripper_controller.h"

Gripper::Gripper()
{

	//Initialize the client for the Action interface to the gripper controller
	//and tell the action client that we want to spin a thread by default
	r_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
	l_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
    
	//wait for the gripper action server to come up
	if (!r_gripper_client_->waitForServer(ros::Duration(5.0)) ||
			!l_gripper_client_->waitForServer(ros::Duration(5.0))){
		ROS_ERROR_STREAM("Failed to connect to l/r_gripper_controller/gripper_action action server.");
		connected_to_controller_ = 0;
	} else {
		connected_to_controller_ = 1;
	}
}

Gripper::~Gripper()
{
    delete r_gripper_client_;
    delete l_gripper_client_;
}

void Gripper::feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
	//save the gripper state of the current action
  gripper_state_ = feedback->position;
}

double Gripper::open_gripper(GripperClient* gripper_client_, double force)
{
	//set grippergoal
	pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = Gripper::GRIPPER_MAX_POSITION;
  open.command.max_effort = force;
  
	//send goal to server
  ROS_INFO("Sending open goal");
  gripper_client_->sendGoal(open, GripperClient::SimpleDoneCallback(),
                GripperClient::SimpleActiveCallback(),
                boost::bind(&Gripper::feedbackCb, this, _1));
                
  gripper_client_->waitForResult();
  
  if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The gripper opened!");
  else 
    ROS_ERROR_STREAM(gripper_client_->getState().toString() <<": The gripper failed to open (" << gripper_state_ << ").");
	return gripper_state_;
}

double Gripper::open_r_gripper(double force)
{
	return open_gripper(r_gripper_client_, force);
}

double Gripper::open_l_gripper(double force)
{
	return open_gripper(l_gripper_client_, force);
}

double Gripper::close_gripper(GripperClient* gripper_client_, double force)
{
	//set squeeze goal
	pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
	squeeze.command.position = Gripper::GRIPPER_MIN_POSITION;
	squeeze.command.max_effort = force;
	
	//send goal to server
	ROS_INFO("Sending squeeze goal");
  gripper_client_->sendGoal(squeeze, GripperClient::SimpleDoneCallback(),
                GripperClient::SimpleActiveCallback(),
                boost::bind(&Gripper::feedbackCb, this, _1));	
                
	gripper_client_->waitForResult(ros::Duration(5.5));
	if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("The gripper closed!");
	else
		ROS_INFO_STREAM(gripper_client_->getState().toString() <<": The gripper didn't close completely (" << gripper_state_ << ").");
	return gripper_state_;
}

double Gripper::close_r_gripper(double force)
{
	return close_gripper(r_gripper_client_, force);
}

double Gripper::close_l_gripper(double force)
{
	return close_gripper(l_gripper_client_, force);
}
