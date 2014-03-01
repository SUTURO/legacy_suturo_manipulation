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

	geometry_msgs::PoseStamped targetPose_;

	bool inCollision_;
	
	
	/**
	 * 
	 * 
	 */
	void subscriberCb(const geometry_msgs::PoseStamped& robotPoseFB);
	
	/**
	 * Calculates the distance between the base_foortprint and the environment.
	 * If the distance is smaller then "footprint_radius", "inCollision_" will 
	 * be set to true;
	 * 
	 */	
	void subscriberCbLaserScan(const sensor_msgs::LaserScan& scan);


  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool checkXCoord(geometry_msgs::PoseStamped targetPose);

  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool checkYCoord(geometry_msgs::PoseStamped targetPose);

  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool checkOrientation(tf::Quaternion q2, tf::Quaternion q3);

  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool checkLocalization();

  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool transformToBaseLink(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink);
	
public:


	Suturo_Manipulation_Move_Robot(ros::NodeHandle* nh);

	~Suturo_Manipulation_Move_Robot();
 	
  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool rotateBase();

  /**
	 * 
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool driveBase(geometry_msgs::PoseStamped targetPose);

  /**
	 * Checks if the given pose is in collision with a collisionobject.
	 * 
	 * @return true, if successfull
	 * 					false, otherwise
	 */
	bool checkCollision(geometry_msgs::PoseStamped targetPose);

	/**
	 * @return inCollision_
	 */	
	bool getInCollision();
  
};

#endif
