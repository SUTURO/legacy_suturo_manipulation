#ifndef SUTURO_MANIPULATION_GRASPING
#define SUTURO_MANIPULATION_GRASPING

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group.h>

#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>


class Grasping
{
private:

	const static std::string LEFT_ARM;
	const static std::string RIGHT_ARM;
	
	const static double cylinder_safty_dist = 0.08;//m
	
	const static int x_side_graspable = 2;
	const static int y_side_graspable = 3;
	const static int z_side_graspable = 5;


	move_group_interface::MoveGroup* group_r_arm_;
	move_group_interface::MoveGroup* group_l_arm_;
	Gripper* gripper_;
	Suturo_Manipulation_Planning_Scene_Interface* pi_;
	tf::TransformBroadcaster br_;
  tf::Transform transform_;

	/**
	 * Updated a Collisionobject with box shape depending on the gripper position.
	 * Uses graspable_sides and pos_id to identify which side has been grasped.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */
	int updateGraspedBoxPose(moveit_msgs::CollisionObject &co, int graspable_sides, int pos_id, double gripper_pose);

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
	 * @return >1, if succesfull
	 * 					@return % 2 == 0, if the object can be grasped on the x side
	 * 					@return % 3 == 0, if the object can be grasped on the y side
	 * 					@return % 5 == 0, if the object can be grasped on the z side
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
				std::vector<geometry_msgs::PoseStamped> &poses, std::vector<geometry_msgs::PoseStamped> &pre_poses, 
				double force, int graspable_sides);

	/**
	 * Calculates grasp und pregrasp position for a box or cylinder.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);

	/**
	 * Publishes/updates a tf frame inside the collisionobject
	 */
	void publishTfFrame(moveit_msgs::CollisionObject co);
	
	/**
	 * Adds Grasppositions possible from above and below, to the vactors.
	 */
	void addBoxGraspPositionsZ(double z, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);
				
	/**
	 * Adds Grasppositions possible from the left and right, to the vactors.
	 */				
	void addBoxGraspPositionsY(double y, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);

	/**
	 * Adds Grasppositions possible from the front and behind, to the vactors.
	 */
	void addBoxGraspPositionsX(double x, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);
				
	void addCylinderGraspPositionsX(double h, double r, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);
	
	
	void addCylinderGraspPositionsY(double h, double r, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses);
	
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
	 * Drops an object.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int dropObject(std::string object_name);
	
	/**
	 * Detaches, whatever is attached to the arm and opens the gripper.
	 * 
	 * @return 1, if succesfull
	 * 					0, otherwise
	 */	
	int drop(std::string arm);

};
     
#endif
