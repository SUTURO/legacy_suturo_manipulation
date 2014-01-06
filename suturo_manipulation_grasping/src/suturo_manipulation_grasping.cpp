#include "suturo_manipulation_grasping.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_gripper_controller.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <suturo_manipulation_planning_scene_interface.h>

using namespace std;

Grasping::Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi)
{
	group_r_arm_ = new move_group_interface::MoveGroup("right_arm");
	group_r_arm_->setPlanningTime(30.0);
	
	group_l_arm_ = new move_group_interface::MoveGroup("left_arm");
	group_l_arm_->setPlanningTime(30.0);
	
	gripper_ = new Gripper();
	pi_ = pi;
}

Grasping::~Grasping()
{

}

int Grasping::calcBoxGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose)
{
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX) return 0;
	return 1;
}

int Grasping::calcBoxGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose)
{
	return 1;
}

int Grasping::calcCylinderGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose)
{
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
		//fehlermeldung
		return 0;
	}
	double h = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
  double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
	
	if (r*2 > Gripper::GRIPPER_MAX_POSITION){
		//fehlermeldung
		return 0;
	}
	
	
	return 1;
}

int Grasping::calcCylinderGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose)
{
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
		ROS_ERROR_STREAM("Wenn der Fehler auftaucht hat Simon Mist gebaut!");
		return 0;
	}
	
	double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
	if (r*2 > Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("Object to big!");
		return 0;
	}
	
	pose.header.frame_id = "/base_footprint";
	
	//copy position of object
	pose.pose.position = co.primitive_poses[0].position;
	
	//choose default orientation for hand
	pose.pose.orientation.w = 1;
	
	//grab object form the front
	pose.pose.position.x -= Gripper::GRIPPER_DEPTH;
	
	pre_pose.header.frame_id = "/base_footprint";
	pre_pose = pose;
	pre_pose.pose.position.x -= Gripper::GRIPPER_DEPTH;
	
	return 1;
}

int Grasping::calcGraspPosition(std::string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose)
{
	//get object from planningscene
	moveit_msgs::CollisionObject co;
	if (!pi_->getObject(objectName, co)){
		//errormessage printed by getObject
		return 0;
	}
	
	//choose the right grasppositoncalculationfunction
	switch (co.primitives[0].type){
		case shape_msgs::SolidPrimitive::CYLINDER:
			calcCylinderGraspPositionGammelig(co, pose, pre_pose);
			break;
		
		case shape_msgs::SolidPrimitive::BOX:
			calcBoxGraspPositionGammelig(co, pose, pre_pose);
			break;
		
		default: 
			ROS_ERROR_STREAM("Can't calculate grasppositon for objecttype: " << co.primitives[0].type);
			break;
	}

	return 1;
}

int Grasping::r_arm_pick(std::string objectName)
{
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	if (!calcGraspPosition(objectName, pose, pre_pose)){
		//errormessage printed by calcGraspPosition
		return 0;
	}
	return r_arm_pick(objectName, pose, pre_pose);
}

int Grasping::r_arm_pick(string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose)
{
	return pick(objectName, R_ARM, pose, pre_pose);
}

int Grasping::l_arm_pick(std::string objectName)
{
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	if (!calcGraspPosition(objectName, pose, pre_pose)){
		//errormessage printed by calcGraspPosition
		return 0;
	}
	return l_arm_pick(objectName, pose, pre_pose);
}


int Grasping::l_arm_pick(string objectName, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose)
{
	return pick(objectName, L_ARM, pose, pre_pose);
}

int Grasping::pick(string objectName, int arm, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose)
{
	move_group_interface::MoveGroup group(*group_r_arm_);
	if (arm == L_ARM){
		group = *group_l_arm_;
	}
	
	//go into pregraspposition
	group.setPoseTarget(pre_pose);
	group.move();
	
	//open gripper
	if (arm == R_ARM){
		gripper_->open_r_gripper();
	}else{
		gripper_->open_l_gripper();
	}
	
	//set goal to pose
	group.setPoseTarget(pose);
	
	//move Arm to goalpose
	if (!group.move()){
		ROS_ERROR_STREAM("Failed to move to " << objectName << " at: " << pose);
		 return 0;
	 }
	
	//close grapper
	if (arm == R_ARM){
		gripper_->close_r_gripper();
	}else{
		gripper_->close_l_gripper();
	}
	
	//attach object
	pi_->attachObject(objectName, group.getEndEffectorLink());
	
	//lift object
	geometry_msgs::PoseStamped pose2 = pose;
	pose2.pose.position.z += 0.05;
	group.setPoseTarget(pose2);
	group.move();
	
	return 1;
}

int Grasping::drop(string objectName)
{
	//get object from planningscene
	
	//find arm that holds the object
	
	//open gripper
	
	//detach object
	return 0;
}
