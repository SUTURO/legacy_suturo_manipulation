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

const string Grasping::R_ARM = "right_arm";
const string Grasping::L_ARM = "left_arm";

Grasping::Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi)
{
	group_r_arm_ = new move_group_interface::MoveGroup("right_arm");
	group_r_arm_->setPlanningTime(20.0);
	
	group_l_arm_ = new move_group_interface::MoveGroup("left_arm");
	group_l_arm_->setPlanningTime(20.0);
	
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
	//test if the object is a box
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX){
		ROS_ERROR_STREAM("Wenn der Fehler auftaucht hat Simon Mist gebaut!");
		return 0;
	}
	
	//test object size
	double x = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
  double y = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
  double z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
	if (x > Gripper::GRIPPER_MAX_POSITION &&
			y > Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("Object is to big!");
		return 0;
	}
	
	pose.header.frame_id = "/base_footprint";
	
	//copy position of object
	pose.pose.position = co.primitive_poses[0].position;
	
	//choose default orientation for hand
	geometry_msgs::Quaternion box_orientation = co.primitive_poses[0].orientation;
	
	
	if (y > Gripper::GRIPPER_MAX_POSITION){
		//rotate gripper to grap the y side
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, tf::getYaw(box_orientation)+M_PI_2);
	} else {
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, tf::getYaw(box_orientation));
	}
	
	ROS_INFO_STREAM(tf::getYaw(box_orientation));
	
	//grab object form the front
	pose.pose.position.z += Gripper::GRIPPER_DEPTH + z/2;
	
	
	pre_pose.header.frame_id = "/base_footprint";
	pre_pose = pose;
	pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH;
	
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
	//test if object is a cylinder
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
		ROS_ERROR_STREAM("Wenn der Fehler auftaucht hat Simon Mist gebaut!");
		return 0;
	}
	
	//test objectsize
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
	pose.pose.position.x -= Gripper::GRIPPER_DEPTH + r;
	
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

int Grasping::pick(string objectName, std::string arm, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose)
{
	move_group_interface::MoveGroup group(*group_r_arm_);
	if (arm == L_ARM){
		group = *group_l_arm_;
	}
	
	//go into pregraspposition
	group.setPoseTarget(pre_pose);
	if (!group.move()){
		ROS_ERROR_STREAM("Failed to move to pregraspposition of " << objectName << " at: " << pose);
		return 0;		
	}
	
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
	pi_->attachObject(objectName, group.getEndEffectorLink(), Gripper::get_r_gripper_links());
	
	//lift object
	geometry_msgs::PoseStamped pose2 = pose;
	pose2.pose.position.z += 0.05;
	group.setPoseTarget(pose2);
	group.move();
	
	return 1;
}

int Grasping::pick(std::string objectName, std::string arm)
{
	if (arm == R_ARM){
		return r_arm_pick(objectName);
	} else {
		return l_arm_pick(objectName);
	}
}


int Grasping::drop(string objectName)
{
	//get object from planningscene
	moveit_msgs::AttachedCollisionObject aco;
	if (pi_->getAttachedObject(objectName, aco))
	{
		ROS_INFO("Become object");
	} else {
		ROS_INFO("No object found!");
	}
	//find arm that holds the object
	bool r_grasp;
	bool l_grasp;

	r_grasp = aco.link_name == group_r_arm_->getEndEffectorLink();
	l_grasp = aco.link_name == group_l_arm_->getEndEffectorLink();

	//open gripper
	if (r_grasp && l_grasp){
		gripper_->open_r_gripper();
		gripper_->open_l_gripper();
	} else if(r_grasp) {
		gripper_->open_r_gripper();
	} else {
		gripper_->open_l_gripper();
	}	

	//detach object
	pi_->detachObject(objectName);
	return 0;
}
