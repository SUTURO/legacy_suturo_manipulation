#include "suturo_manipulation_gripper_controller.h"

const std::string Gripper::RIGHT_ARM = suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM;
const std::string Gripper::LEFT_ARM = suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM;

const std::string Gripper::R_GRIPPER_TOPIC = "r_gripper_controller/gripper_action";
const std::string Gripper::L_GRIPPER_TOPIC = "l_gripper_controller/gripper_action";

Gripper::Gripper(std::string arm)
{
    arm_ = arm;
    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    if (arm_ == RIGHT_ARM)
    {
        gripper_client_ = new GripperClient(R_GRIPPER_TOPIC, true);
    }
    else
    {
        gripper_client_ = new GripperClient(L_GRIPPER_TOPIC, true);
    }

    //wait for the gripper action server to come up
    if (!gripper_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR_STREAM("Failed to connect to Gripper action server.");
        connected_to_controller_ = 0;
    }
    else
    {
        connected_to_controller_ = 1;
    }
}

Gripper::~Gripper()
{
    delete gripper_client_;
}

void Gripper::feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr &feedback)
{
    //save the gripper state of the current action
    gripper_state_ = feedback->position;
}

double Gripper::open_gripper(double force)
{
    if (!is_connected_to_controller())
    {
        ROS_ERROR_STREAM("not connected to grippercontroller");
        return false;
    }
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

    if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The gripper opened!");
    else
        ROS_ERROR_STREAM(gripper_client_->getState().toString() << ": The gripper failed to open (" << gripper_state_ << ").");
    return gripper_state_;
}

double Gripper::close_gripper(double force)
{
    if (!is_connected_to_controller())
    {
        ROS_ERROR_STREAM("not connected to grippercontroller");
        return false;
    }
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
    if (gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The gripper closed!");
    else
        ROS_INFO_STREAM(gripper_client_->getState().toString() << ": The gripper didn't close completely (" << gripper_state_ << ").");
    return gripper_state_;
}

double Gripper::get_gripper_palm_length()
{
	double r = Gripper::R_GRIPPER_PALM_LENGTH;
	double l = Gripper::L_GRIPPER_PALM_LENGTH;
    return arm_ == RIGHT_ARM ? r : l;
}

std::vector<std::string> Gripper::get_gripper_links()
{
    return arm_ == RIGHT_ARM ? Gripper::get_r_gripper_links() : Gripper::get_r_gripper_links();
}