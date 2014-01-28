/**
* This class implements the action server to move a
* part of the robot (left_arm, right_arm, head)
* to a definied home position.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_homeAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <control_msgs/PointHeadActionGoal.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_homeAction> Server;

control_msgs::PointHeadActionGoal goal_msg;

bool moved = 0;

template <class T>
/**
* This method formats a ros time to a string.
* Thanks to https://code.ros.org/trac/ros/ticket/2030
*/
std::string time_to_str(T ros_t)
{
  char buf[1024]      = "";
  time_t t = ros_t.sec;
  struct tm *tms = localtime(&t);
  strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
  return std::string(buf);
}

/**
* This method is a callback method and checks, if the head moved.
*/
void moveHeadResult(pr2_controllers_msgs::PointHeadActionResult msg)
{
  if(goal_msg.goal_id.id == msg.status.goal_id.id){
  	ROS_INFO("move_home_server: Get result!");
  	moved = 1;
  } else {
  	ROS_INFO("move_home_server: No result!");
  	moved = 0;
  }
}

/**
* This method moves the given bodypart to the homeposition.
*/
void moveHome(const suturo_manipulation_msgs::suturo_manipulation_homeGoalConstPtr& goal, 
			ros::Publisher* publisher, Server* server_home)
{	
	ROS_INFO("callback moveHome() begins...");

	// create a move result message
	suturo_manipulation_msgs::suturo_manipulation_homeResult r;	
	// Set part which should be go home
	string body_part = goal->bodypart.bodyPart;
	ROS_INFO("Bodypart: %s", body_part.c_str());
	// Set header
	r.succ.header.stamp = ros::Time();
	// Set Answer for planning to undefined
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;
	
	// if(body_part==suturo_manipulation_msgs::RobotBodyPart::HEAD){
	// 	// bodypart = head
	// 	ROS_INFO("Move head home!");
	// 	// Create headHome object
	// 	geometry_msgs::PoseStamped headHome;

	// 	// Set home Coordinates, time and frame
	// 	headHome.pose.position.x = 1;
	// 	headHome.pose.position.y = 0;
	// 	headHome.pose.position.z = 0;
	// 	headHome.header.stamp = ros::Time::now();
	// 	headHome.header.frame_id = "/torso_lift_link";

 //        // Publish goal on topic /suturo/head_controller
 //        if( !publisher ) {
 //        	ROS_INFO("Publisher invalid!\n");
 //        	r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
 //          	server_home->setAborted(r);
 //        } else {
 //            publisher->publish(headHome);
 //            ROS_INFO("Home Goal published!\n");
 //            r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
 //            server_home->setSucceeded(r);
 //        }
	// }

	if(body_part==suturo_manipulation_msgs::RobotBodyPart::HEAD){
		// bodypart = head
		ROS_INFO("Move head home!");

		// Set home Coordinates, time and frame
		goal_msg.header.seq = 1;
	    goal_msg.header.stamp = ros::Time::now();
	    goal_msg.header.frame_id = "/torso_lift_link";
	    goal_msg.goal_id.stamp = goal_msg.header.stamp;
	    // set unique id with timestamp
	    goal_msg.goal_id.id = "goal_"+time_to_str(goal_msg.header.stamp);
	    goal_msg.goal.target.header = goal_msg.header;
	    goal_msg.goal.target.point.x = 1;
	    goal_msg.goal.target.point.y = 0;
	    goal_msg.goal.target.point.z = 0;
	    goal_msg.goal.pointing_axis.x = 1;
	    goal_msg.goal.pointing_axis.y = 0;
	    goal_msg.goal.pointing_axis.z = 0;
	    goal_msg.goal.pointing_frame = "head_plate_frame";
	    goal_msg.goal.min_duration = ros::Duration(1.0);
	    goal_msg.goal.max_velocity = 10;

		// Publish goal on topic /suturo/head_controller
		if( !publisher ) {
			ROS_INFO("Publisher invalid!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		} else {
			publisher->publish(goal_msg);
			ROS_INFO("Home Goal published!");
			ros::WallDuration(2.0).sleep();
			if(moved == 1){
				ROS_INFO("Head moved!\n");
				r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
				server_home->setSucceeded(r);
			} else {
				ROS_INFO("Head doesn't move!\n");
				r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		  		server_home->setAborted(r);
			}
		}
	} else if ((body_part==suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM)){
		// bodypart = left_arm
		ROS_INFO("Move left_arm home!");
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
			ROS_INFO("Moved home!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_home->setSucceeded(r);
		} else {
			ROS_INFO("Moving home failed!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	} else if ((body_part==suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM)){
		// bodypart = right_arm
		ROS_INFO("Move right_arm home!");
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
			ROS_INFO("Moved home!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_home->setSucceeded(r);
		} else {
			ROS_INFO("Moving home failed!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	} else if ((body_part==suturo_manipulation_msgs::RobotBodyPart::BOTH_ARMS)){
		// bodypart = both_arms
		ROS_INFO("Move both arms home!");
		// set group to move
		move_group_interface::MoveGroup group(body_part);
		
		// set group name to go home
		group.setNamedTarget(body_part+"_home");
		
		//move bodypart
		if (group.move()){
			ROS_INFO("Moved home!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
			server_home->setSucceeded(r);
		} else {
			ROS_INFO("Moving home failed!\n");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
			server_home->setAborted(r);
		}
	} else {
		ROS_INFO("Unknown bodypart!\n");
		r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_home->setAborted(r);	
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_move_home_server");
	ros::NodeHandle n;

	// Publish a topic for the head controller
    // ros::Publisher head_publisher = n.advertise<geometry_msgs::PoseStamped>("/suturo/head_controller_goal_point", 1000);

	// Subscribe the topic, call moveHeadResult if data is published
	ros::Subscriber sub = n.subscribe("/head_traj_controller/point_head_action/result", 1000, moveHeadResult);

	// Publish a topic for the ros intern head controller
	ros::Publisher head_publisher = n.advertise<control_msgs::PointHeadActionGoal>("/head_traj_controller/point_head_action/goal", 1000);


	// create the action server
	Server server_home(n, "suturo_man_move_home_server", boost::bind(&moveHome, _1, &head_publisher, &server_home), false);
	// start the server
	server_home.start();
	
	ROS_INFO("Ready to go home!");
	ros::spin();
	return 0;
}
