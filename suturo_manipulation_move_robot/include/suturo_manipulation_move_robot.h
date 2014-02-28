#ifndef SUTURO_MANIPULATION_MOVE_ROBOT
#define SUTURO_MANIPULATION_MOVE_ROBOT

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <sensor_msgs/LaserScan.h>


class Suturo_Manipulation_Move_Robot{
	
private:
  const static double footprint_radius = 0.5; //0.47

	//! The node handle we'll be using
	ros::NodeHandle* nh_;
	//! We will be publishing to the "/base_controller/command" topic to issue commands
	ros::Publisher cmd_vel_pub_;
	// Localisation subscriber
	ros::Subscriber loc_sub_;
	
	ros::Subscriber collision_sub_;
	// robot position
	geometry_msgs::PoseStamped robotPose_;

	geometry_msgs::Twist base_cmd_;

	tf::TransformListener listener_;
	
	Suturo_Manipulation_Planning_Scene_Interface* pi_;

	bool inCollision_;
	tfScalar robotAngles_;
	tfScalar targetAngles_;
	/**
	 * 
	 */
	void subscriberCb(const geometry_msgs::PoseStamped& robotPoseFB);
	
	
	void subscriberCbLaserScan(const sensor_msgs::LaserScan& scan);



	bool checkXCoord(geometry_msgs::PoseStamped targetPose);

	bool checkYCoord(geometry_msgs::PoseStamped targetPose);

	bool checkOrientation(tf::Quaternion q2, tf::Quaternion q3);

	bool checkLocalization();

	bool transformToBaseLink(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink);
	
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

	bool checkCollision(geometry_msgs::PoseStamped targetPose);
	
	bool getInCollision();
  
};

#endif
