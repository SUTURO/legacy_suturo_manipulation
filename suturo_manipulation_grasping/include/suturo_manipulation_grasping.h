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

	move_group_interface::MoveGroup* group_r_arm_;
	move_group_interface::MoveGroup* group_l_arm_;
	Gripper* gripper_;
	Suturo_Manipulation_Planning_Scene_Interface* pi_;

	int updateGraspedBoxPose(moveit_msgs::CollisionObject &co, std::string arm);
	
	int updateGraspedCylinderPose(moveit_msgs::CollisionObject &co, std::string arm);

	int calcBoxGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);
	
	int calcBoxGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);

	int calcCylinderGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);
	
	int calcCylinderGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose);
	
	int pick(moveit_msgs::CollisionObject co, std::string arm, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose, double force);

public:
	
	Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi);

	~Grasping();

	int calcGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose);

	int pick(std::string objectName, std::string arm, double force=50.0);
	
	int drop(std::string objectName);
};
     
#endif
