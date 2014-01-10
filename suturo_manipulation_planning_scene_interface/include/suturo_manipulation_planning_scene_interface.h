#ifndef SUTURO_MANIPULATION_PLANNING_SCENE_INTERFACE
#define SUTURO_MANIPULATION_PLANNING_SCENE_INTERFACE

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetPlanningScene.h>


class Suturo_Manipulation_Planning_Scene_Interface{
private:
	ros::NodeHandle* nh_; 
	ros::Publisher attached_object_publisher_;

public:
	Suturo_Manipulation_Planning_Scene_Interface(ros::NodeHandle* nodehandle);

	~Suturo_Manipulation_Planning_Scene_Interface();
	
	int attachObject(std::string objectName, std::string linkName,
							std::vector<std::string> gripper_links);
	
	int detachObject(std::string objectName);
	
	int getObject(std::string objectName, moveit_msgs::CollisionObject &co);
	
	int getPlanningScene(moveit_msgs::PlanningScene &ps);	
	
	std::vector<moveit_msgs::AttachedCollisionObject> getAttachedObjects();
	
	int getAttachedObject(std::string objectName, moveit_msgs::AttachedCollisionObject &co);
};
     
#endif
