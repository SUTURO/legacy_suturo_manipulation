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

	const static std::string LEFT_ARM;
	const static std::string RIGHT_ARM;


	move_group_interface::MoveGroup* group_r_arm_;
	move_group_interface::MoveGroup* group_l_arm_;
	Gripper* gripper_;
	Suturo_Manipulation_Planning_Scene_Interface* pi_;

	/**
	 * Updated a Collisionobject with box shape depending on the gripper position.
	 * @param 	co: 	
	 * 						the collisionObject to be updated
	 * 					arm:	
	 * 						the arm that holds the object
	 * 					gripper_pose:
	 * 						the position of the gripper
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */
	int updateGraspedBoxPose(moveit_msgs::CollisionObject &co, geometry_msgs::PoseStamped gripperPose, double gripper_pose);

	/**
	 * Updated a Collisionobject with cylinder shape depending on the gripper position.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int updateGraspedCylinderPose(moveit_msgs::CollisionObject &co, geometry_msgs::PoseStamped gripperPose, double gripper_pose);


	/**
	 * Calculates grasp und pregrasp position for a box.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);

	/**
	 * Calculates grasp und pregrasp position for a cylinder.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);

	/**
	 * Picks an object.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */		
	int pick(moveit_msgs::CollisionObject co, std::string arm, 
				std::vector<geometry_msgs::PoseStamped> &poses, std::vector<geometry_msgs::PoseStamped> &pre_poses, double force);

	/**
	 * Calculates grasp und pregrasp position for a box or cylinder.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);

	void publishTfFrame(moveit_msgs::CollisionObject co);
	
public:
	
	Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi);

	~Grasping();
	
	/**
	 * Picks an object.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int pick(std::string object_name, std::string arm, double force=50.0);
	
	/**
	 * drops an object.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int dropObject(std::string object_name);
	
	int drop(std::string arm);

	int handoff(std::string objectName, std::string new_arm, double force);
};
     
#endif
