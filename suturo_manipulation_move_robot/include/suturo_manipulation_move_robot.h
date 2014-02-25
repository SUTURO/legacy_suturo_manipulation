#ifndef SUTURO_MANIPULATION_MOVE_ROBOT
#define SUTURO_MANIPULATION_MOVE_ROBOT

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


class Suturo_Manipulation_Move_Robot{
	
private:
	//! The node handle we'll be using
	ros::NodeHandle* nh_;
	//! We will be publishing to the "/base_controller/command" topic to issue commands
	ros::Publisher cmd_vel_pub_;
	// Localisation subscriber
	ros::Subscriber loc_sub_;
	// robot position
	geometry_msgs::PoseStamped robotPose_;

	geometry_msgs::Twist base_cmd_;

	/**
	 * 
	 */
	void subscriberCb(const geometry_msgs::PoseWithCovarianceStamped& robotPoseFB);

	bool checkCollision(geometry_msgs::PoseStamped targetPose);

	bool checkXCoord(geometry_msgs::PoseStamped targetPose);

	bool checkYCoord(geometry_msgs::PoseStamped targetPose);

	bool checkOrientation(geometry_msgs::PoseStamped targetPose);
	
public:


	Suturo_Manipulation_Move_Robot(ros::NodeHandle* nh);

	~Suturo_Manipulation_Move_Robot();
 	
 	/**
	 * 
	 * 
	 * @return 
	 */
	bool rotateBase();

	/**
	 * 
	 * 
	 * @return 
	 */
	bool driveBase(geometry_msgs::PoseStamped targetPose);


  
};

#endif
