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

int Grasping::calcBoxGraspPosition(geometry_msgs::PoseStamped &pose, moveit_msgs::CollisionObject co)
{
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX) return 0;
	return 1;
}

int Grasping::calcCylinderGraspPosition(geometry_msgs::PoseStamped &pose, moveit_msgs::CollisionObject co)
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

int Grasping::calcGraspPosition(geometry_msgs::PoseStamped &pose, string objectName)
{
	//get object from planningscene
	moveit_msgs::CollisionObject co2;
	if (!pi_->getObject(objectName, co2)) return 0;
	
	//if (co2.)

	return 1;
}

int Grasping::r_arm_pick(string objectName, geometry_msgs::PoseStamped &pose)
{
	return pick(objectName, R_ARM, pose);
}

int Grasping::l_arm_pick(string objectName, geometry_msgs::PoseStamped &pose)
{
	return pick(objectName, L_ARM, pose);
}

int Grasping::pick(string objectName, int arm, geometry_msgs::PoseStamped &pose)
{
	move_group_interface::MoveGroup group(*group_r_arm_);
	if (arm == L_ARM){
		group = *group_l_arm_;
	}
	
	//go into pregraspposition
	geometry_msgs::PoseStamped pre_pose(pose);
	pre_pose.pose.position.x -= 0.1;
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
		ROS_INFO_STREAM("Failed to move to " << objectName << " at: " << pose);
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
