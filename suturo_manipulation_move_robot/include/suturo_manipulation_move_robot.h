#ifndef SUTURO_MANIPULATION_MOVE_ROBOT
#define SUTURO_MANIPULATION_MOVE_ROBOT

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


class MoveRobot{
	
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "/base_controller/command" topic to issue commands
	ros::Publisher cmd_vel_pub_;
	// Localisation subscriber
	ros::Subscriber loc_sub_;
	// robot position
	geometry_msgs::PoseStamped robotPose_;

	/**
	 * 
	 */
	void subscriberCb(const geometry_msgs::PoseWithCovarianceStamped& robotPoseFB);

	bool checkCollision(geometry_msgs::PoseStamped targetPose);
	
public:


	MoveRobot(ros::NodeHandle &nh);

	~MoveRobot();
 
	/**
	 * 
	 * 
	 * @return 
	 */
	bool driveBase(geometry_msgs::PoseStamped targetPose);


  
};

#endif
