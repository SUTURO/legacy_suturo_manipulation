#ifndef SUTURO_MANIPULATION_GRASPING
#define SUTURO_MANIPULATION_GRASPING

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_planning_scene_interface.h>

class Grasping
{
private:
	static const std::string R_ARM ;
	static const std::string L_ARM ;
	move_group_interface::MoveGroup* group_r_arm_;
	move_group_interface::MoveGroup* group_l_arm_;
	Gripper* gripper_;
	Suturo_Manipulation_Planning_Scene_Interface* pi_;

	int calcBoxGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);
	
	int calcBoxGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);

	int calcCylinderGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);
	
	int calcCylinderGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);
	
	int pick(std::string objectName, std::string arm, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);
	
public:
	Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi);

	~Grasping();

	int calcGraspPosition(std::string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);

	int pick(std::string objectName, std::string arm);

	int r_arm_pick(std::string objectName);
	int r_arm_pick(std::string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);

	int l_arm_pick(std::string objectName);
	int l_arm_pick(std::string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);
	
	int drop(std::string objectName);
};
     
#endif
