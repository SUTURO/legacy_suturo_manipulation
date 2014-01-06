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
	static const int R_ARM = 0;
	static const int L_ARM = 1;
	move_group_interface::MoveGroup* group_r_arm_;
	move_group_interface::MoveGroup* group_l_arm_;
	Gripper* gripper_;
	Suturo_Manipulation_Planning_Scene_Interface* pi_;

	int calcBoxGraspPosition(geometry_msgs::PoseStamped &pose, moveit_msgs::CollisionObject co);
	
	int calcCylinderGraspPosition(geometry_msgs::PoseStamped &pose, moveit_msgs::CollisionObject co);
	
	int calcCylinderGraspPositionGammelig(geometry_msgs::PoseStamped &pose, moveit_msgs::CollisionObject co);
	
	int pick(std::string objectName, int arm, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);
	
public:
	Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi);

	~Grasping();

	int calcGraspPosition(geometry_msgs::PoseStamped &pose, std::string objectName);

	int r_arm_pick(std::string objectName);
	int r_arm_pick(std::string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);

	int l_arm_pick(std::string objectName);
	int l_arm_pick(std::string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);
	
	int drop(std::string objectName);
};
     
#endif
