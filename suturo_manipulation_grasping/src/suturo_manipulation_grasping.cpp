#include "suturo_manipulation_grasping.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_gripper_controller.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>

using namespace std;

Grasping::Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi)
{
	group_r_arm_ = new move_group_interface::MoveGroup(suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
	group_r_arm_->setPlanningTime(20.0);
	
	group_l_arm_ = new move_group_interface::MoveGroup(suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM);
	group_l_arm_->setPlanningTime(20.0);
	//~ ROS_INFO_STREAM(group_l_arm_->getPlanningFrame());
	//~ group_l_arm_->setWorkspace(1, 1, 1.5, -1, 0, 0);
	gripper_ = new Gripper();
	pi_ = pi;
}

Grasping::~Grasping()
{

}

int Grasping::calcBoxGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose)
{
	ROS_INFO_STREAM("calculate graspposition for " << co.id);
	
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
	
	pose.header.frame_id = co.header.frame_id;
	
	//copy position of object
	pose.pose.position = co.primitive_poses[0].position;
	
	//choose default orientation for hand
	geometry_msgs::Quaternion box_orientation = co.primitive_poses[0].orientation;
	
	
	if (y > Gripper::GRIPPER_MAX_POSITION){
		//rotate gripper to grap the y side
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, tf::getYaw(box_orientation)+M_PI_2);
		ROS_INFO_STREAM("gripper yaw1 " << tf::getYaw(box_orientation)+M_PI_2);
		ROS_INFO_STREAM("pose yaw1 " << tf::getYaw(pose.pose.orientation));
	} else {
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, tf::getYaw(box_orientation));
		ROS_INFO_STREAM("gripper yaw1 " << tf::getYaw(box_orientation));
		ROS_INFO_STREAM("pose yaw1 " << tf::getYaw(pose.pose.orientation));
	}
	
	//grab object form the front
	pose.pose.position.z += Gripper::GRIPPER_DEPTH + z/2;
	
	
	pre_pose.header.frame_id = co.header.frame_id;
	pre_pose = pose;
	pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH;
	
	return 1;
}

int Grasping::calcCylinderGraspPositionGammelig(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, 
				geometry_msgs::PoseStamped &pre_pose)
{
	ROS_INFO_STREAM("calculate graspposition for " << co.id);
	//test if object is a cylinder
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
		ROS_ERROR_STREAM("Wenn der Fehler auftaucht hat Simon Mist gebaut!");
		return 0;
	}
	
	//test objectsize
	double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
	if (r*2 > Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("Object is to big!");
		return 0;
	}
	
	pose.header.frame_id = co.header.frame_id;
	
	//copy position of object
	pose.pose.position = co.primitive_poses[0].position;
	
	//choose default orientation for hand
	pose.pose.orientation.w = 1;
	
	//grab object form the front
	pose.pose.position.x -= Gripper::GRIPPER_DEPTH + r;
	
	pre_pose.header.frame_id = co.header.frame_id;
	pre_pose.pose.position.x = pose.pose.position.x - Gripper::GRIPPER_DEPTH;
	pre_pose.pose.position.y = pose.pose.position.y;
	pre_pose.pose.position.z = pose.pose.position.z;
	//~ pre_pose.pose.position.x -= Gripper::GRIPPER_DEPTH+0.25;
	
	return 1;
}

int Grasping::calcGraspPosition(moveit_msgs::CollisionObject co, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose)
{
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

int Grasping::updateGraspedBoxPose(moveit_msgs::CollisionObject &co, std::string arm)
{
	if (co.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
		//wir greifen eh nur boxen..
		return 1;
	}
	
	geometry_msgs::PoseStamped gripperPose;
	std::vector<double> rpy;
	double gripper_open_kack_name;
	
	if (arm == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){
		gripperPose = group_r_arm_->getCurrentPose();
		rpy = group_r_arm_->getCurrentRPY();
		
		move_group_interface::MoveGroup group("right_gripper");
		gripper_open_kack_name = group.getCurrentJointValues().at(3);
	} else if (arm == suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM){
		gripperPose = group_l_arm_->getCurrentPose();
		rpy = group_l_arm_->getCurrentRPY();
		
		move_group_interface::MoveGroup group("left_gripper");
		gripper_open_kack_name = group.getCurrentJointValues().at(3);
	} else {
		ROS_ERROR("wrong arm parameter.");
		return 0;
	}
	
	double y = co.primitive_poses[0].position.y;
	double z = co.primitive_poses[0].position.z;
	
	//update object position based on gripperposition
	//~ co.primitive_poses[0].position = gripperPose.pose.position;
	//~ co.primitive_poses[0].position.z -= Gripper::GRIPPER_DEPTH + z/2 - 0.005;
	
	//update object size based on gripperstate
	if (y > Gripper::GRIPPER_MAX_POSITION){
		//x achse wurde gegriffen
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = gripper_open_kack_name-0.005;
	} else {
		//y achse wurde gegriffen
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = gripper_open_kack_name-0.005;
	}
	
	return 1;
}

int Grasping::pick(moveit_msgs::CollisionObject co, std::string arm, geometry_msgs::PoseStamped &pose, geometry_msgs::PoseStamped &pre_pose, double force)
{
	string objectName = co.id;
	move_group_interface::MoveGroup* group;
	if (arm == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){
		group = new move_group_interface::MoveGroup(*group_r_arm_);
	} else if (arm == suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM) {
		group = new move_group_interface::MoveGroup(*group_l_arm_);
	} else {
		ROS_ERROR_STREAM("Grasping::pick| arm value not valide.");
		return 0;
	}
	
	//go into pregraspposition
	ROS_INFO_STREAM("set pregrasppositiontarget");
	group->setPoseTarget(pre_pose);
	ROS_INFO_STREAM("move to pregraspposition");
	if (!group->move()){
		ROS_ERROR_STREAM("Failed to move to pregraspposition of " << objectName << " at: " << pose);
		return 0;		
	}
	
	//open gripper
	if (arm == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){
		gripper_->open_r_gripper();
	}else{
		gripper_->open_l_gripper();
	}
	
	//set goal to pose
	ROS_INFO_STREAM("set goalpose");
	group->setPoseTarget(pose);
	
	//move Arm to goalpose
	ROS_INFO_STREAM("move to goalpose");
	if (!group->move()){
		ROS_ERROR_STREAM("Failed to move to " << objectName << " at: " << pose);
		return 0;
	}
	
	//close grapper
	if (arm == suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){
		gripper_->close_r_gripper(force);
	}else{
		gripper_->close_l_gripper(force);
	}
	
	ROS_INFO_STREAM("update objectposition in planningscene.");
	updateGraspedBoxPose(co, arm);
	pi_->addObject(co);
	
	//attach object
	ROS_INFO_STREAM("attach object");
	pi_->attachObject(objectName, group->getEndEffectorLink(), Gripper::get_r_gripper_links());
	
	//lift object
	ROS_INFO_STREAM("lift object");
	geometry_msgs::PoseStamped pose2 = pose;
	pose2.pose.position.z += 0.05;
	group->setPoseTarget(pose2);
	group->move();
	
	return 1;
}


int Grasping::pick(std::string objectName, std::string arm, double force)
{
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	
	//get object from planningscene
	moveit_msgs::CollisionObject co;
	if (!pi_->getObject(objectName, co)){
		//errormessage printed by getObject
		return 0;
	}
	
	if (!calcGraspPosition(co, pose, pre_pose)){
		//errormessage printed by calcGraspPosition
		return 0;
	}
	
	return pick(co, arm, pose, pre_pose, force);
}


int Grasping::drop(string objectName)
{
	//get object from planningscene
	moveit_msgs::AttachedCollisionObject aco;
	if (pi_->getAttachedObject(objectName, aco))
	{
		ROS_INFO("Get object");
	} else {
		return 0;
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
	} else if (l_grasp){
		gripper_->open_l_gripper();
	}	else {
		ROS_ERROR_STREAM("Grasping::drop| Object probably isn't attached to any hand.");
	}
	
	//detach object
	pi_->detachObject(objectName);
	ROS_INFO_STREAM("droped " << objectName << " successfully.");
	return 1;
}
