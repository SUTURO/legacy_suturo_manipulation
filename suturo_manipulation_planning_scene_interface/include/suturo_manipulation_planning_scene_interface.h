#ifndef SUTURO_MANIPULATION_PLANNING_SCENE_INTERFACE
#define SUTURO_MANIPULATION_PLANNING_SCENE_INTERFACE

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <visualization_msgs/Marker.h>

class Suturo_Manipulation_Planning_Scene_Interface{
private:
	ros::NodeHandle* nh_; 
	ros::Publisher attached_object_publisher_;
	ros::Publisher collision_object_publisher_;
	ros::Publisher vis_pub_;
	ros::Publisher planning_scene_publisher;
	
	ros::ServiceClient ps_service_client_;

public:
	Suturo_Manipulation_Planning_Scene_Interface(ros::NodeHandle* nodehandle);

	~Suturo_Manipulation_Planning_Scene_Interface();
	
	/**
	 * Attach an object to the robot and publish it to the planningscene.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */
	int attachObject(std::string objectName, std::string link_name,
							std::vector<std::string> gripper_links);

	/**
	 * Detach an object from the robot and publish it to the planningscene.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */	
	int detachObject(std::string object_name);
	
	/**
	 * Get an object from the planningscene.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */
	int getObject(std::string object_name, moveit_msgs::CollisionObject &co);
	
	/**
	 * Remove an object (in case it already existed) and add it back to the planningscene.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */
	int addObject(moveit_msgs::CollisionObject co);
	
	/**
	 * Remove an object from the planningscene.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */	
	int removeObject(moveit_msgs::CollisionObject co);
	
	/**
	 * Get the whole planningscene
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */	
	int getPlanningScene(moveit_msgs::PlanningScene &ps);	

	/**
	 * Get a list of all attached objects.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */	
	int getAttachedObjects(std::vector<moveit_msgs::AttachedCollisionObject> &acos);

	/**
	 * Get a list of all collision objects.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */		
	int getObjects(std::vector<moveit_msgs::CollisionObject> &cos);

	/**
	 * Get an attached object.
	 * 
	 * @return 1, if successfull
	 * 					0, otherwise
	 */	
	int getAttachedObject(std::string object_name, moveit_msgs::AttachedCollisionObject &co);
	
	/**
	 * Check if the object is attached to the robot.
	 * 
	 * @return 1, if the object is attached
	 * 					0, otherwise
	 */	
	int isAnObjectAttachedToArm(std::string link_name, moveit_msgs::AttachedCollisionObject &aco);
	
	int allowCollision(std::string object1, std::string object2);

	int denyCollision(std::string object1, std::string object2);
	
	/**
	 * Publish a Marker for rviz at the given pose.
	 * 
	 */	
	void publishMarker(geometry_msgs::PoseStamped pose);
};
     
#endif
