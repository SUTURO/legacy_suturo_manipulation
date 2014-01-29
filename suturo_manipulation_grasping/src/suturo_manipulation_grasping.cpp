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

#include <control_msgs/PointHeadActionGoal.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>

#include <tf/transform_broadcaster.h>

using namespace std;

const string Grasping::RIGHT_ARM = suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM;
const string Grasping::LEFT_ARM = suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM;

Grasping::Grasping(Suturo_Manipulation_Planning_Scene_Interface* pi)
{
	//initialize private variables
	group_r_arm_ = new move_group_interface::MoveGroup(RIGHT_ARM);
	group_r_arm_->setPlanningTime(5.0);
	
	group_l_arm_ = new move_group_interface::MoveGroup(LEFT_ARM);
	group_l_arm_->setPlanningTime(5.0);
	
	
	gripper_ = new Gripper();
	
	//pi nicht selbst erstellen, weil das Weiterreichen des nodehandle über 2 Klassen rumbugt :(
	pi_ = pi;
}

Grasping::~Grasping()
{

}


void Grasping::publishTfFrame(moveit_msgs::CollisionObject co)
{
	tf::TransformBroadcaster br;
  tf::Transform transform;
	ros::WallDuration(1.0).sleep();
  ROS_INFO_STREAM("publish object frame " << co.id);
  transform.setOrigin( tf::Vector3(co.primitive_poses[0].position.x, 
				co.primitive_poses[0].position.y, co.primitive_poses[0].position.z) );
  transform.setRotation( tf::Quaternion(co.primitive_poses[0].orientation.x,
					co.primitive_poses[0].orientation.y,
					co.primitive_poses[0].orientation.z,
					co.primitive_poses[0].orientation.w) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", co.id));
  ros::WallDuration(1.0).sleep();
}

int Grasping::calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses)

{
	ROS_INFO_STREAM("calculate graspposition for " << co.id);
	
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	
	//test if the object is a box
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX){
		ROS_ERROR_STREAM(co.id << " is not a box.");
		return 0;
	}
	
	//get object size
	double x = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
  double y = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
  double z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
  
  //check if the object can be grasped
	if (x > Gripper::GRIPPER_MAX_POSITION &&
			y > Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("Object is to big!");
		return 0;
	}
	
	publishTfFrame(co);

	pose.header.frame_id = co.id;
	pre_pose.header.frame_id = co.id;
	
	//copy position of object
	//~ pose.pose.position = co.primitive_poses[0].position;
	
	//get orientation of the box
	//~ geometry_msgs::Quaternion box_orientation = co.primitive_poses[0].orientation;
	
	//set the gripper orientation:
	//Pitch of M_PI_2 for pointing down
	//Yaw depending on the orientation of the box
	if (y > Gripper::GRIPPER_MAX_POSITION){
		//rotate gripper by a yaw of 90° to grap the y side
	

		//oben
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, M_PI_2);
		
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = Gripper::GRIPPER_DEPTH + z/2;
		pre_pose = pose;
		pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);

		//unten
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, M_PI_2);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;		
		pose.pose.position.z = 0- Gripper::GRIPPER_DEPTH - z/2;
		pre_pose = pose;
		pre_pose.pose.position.z -= Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);
		
		//links
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_2);
		
		pose.pose.position.x = 0;
		pose.pose.position.y = Gripper::GRIPPER_DEPTH + y/2;
		pose.pose.position.z = 0;
		pre_pose = pose;
		pre_pose.pose.position.y += Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);
		
		//rechts		
		
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);
		pose.pose.position.x = 0;
		pose.pose.position.z = 0;		
		pose.pose.position.y = 0 - Gripper::GRIPPER_DEPTH - y/2;
		pre_pose = pose;
		pre_pose.pose.position.y -= Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);

		
	} else {

		//oben
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, 0);
		
		pose.pose.position.z += Gripper::GRIPPER_DEPTH + z/2;
		pre_pose = pose;
		pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);

		//unten
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, 0);
		
		pose.pose.position.z -= Gripper::GRIPPER_DEPTH + z/2;
		pre_pose = pose;
		pre_pose.pose.position.z -= Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);
		
		//links
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);
		
		pose.pose.position.y -= Gripper::GRIPPER_DEPTH + y/2;
		pre_pose = pose;
		pre_pose.pose.position.y -= Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);
		
		//rechts		
		
		pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_2);
		
		pose.pose.position.y -= Gripper::GRIPPER_DEPTH + y/2;
		pre_pose = pose;
		pre_pose.pose.position.y -= Gripper::GRIPPER_DEPTH -0.05;
		
		poses.push_back(pose);
		pre_poses.push_back(pre_pose);
	}
	
	//grap the object from above
	//~ pose.pose.position.z += Gripper::GRIPPER_DEPTH + z/2;
	
	//set pregrasppostion
	
	//~ pre_pose = pose;
	//~ pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH -0.05;
	

	
	return 1;
}

int Grasping::calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
	ROS_INFO_STREAM("calculate graspposition for " << co.id);
	
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	
	//test if object is a cylinder
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
		ROS_ERROR_STREAM(co.id << " is not a cylinder.");
		return 0;
	}
	
	//get objectsize
	double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
	
	//check if the object is too big
	if (r*2 > Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("Object is to big!");
		return 0;
	}
	
	pose.header.frame_id = co.header.frame_id;
	
	//copy position of object
	pose.pose.position = co.primitive_poses[0].position;
	
	//set grasp orientation for hand
	pose.pose.orientation.w = 1;
	
	//grab object form the front
	pose.pose.position.x -= Gripper::GRIPPER_DEPTH + r;
	
	//set pregraspposition
	pre_pose.header.frame_id = co.header.frame_id;
	pre_pose.pose.position.x = pose.pose.position.x - Gripper::GRIPPER_DEPTH;
	pre_pose.pose.position.y = pose.pose.position.y;
	pre_pose.pose.position.z = pose.pose.position.z;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);
	
	return 1;
}

int Grasping::calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
						std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
	//choose the right grasppositoncalculationfunction
	switch (co.primitives[0].type){
		case shape_msgs::SolidPrimitive::CYLINDER:
			return calcCylinderGraspPosition(co, poses, pre_poses);
			break;
		
		case shape_msgs::SolidPrimitive::BOX:
			return calcBoxGraspPosition(co, poses, pre_poses);
			break;
		
		default: 
			ROS_ERROR_STREAM("Can't calculate grasppositon for objecttype: " << co.primitives[0].type);
			return 0;
			break;
	}

	ROS_INFO_STREAM("Something bad happened.");
	return 0;
}

int Grasping::updateGraspedCylinderPose(moveit_msgs::CollisionObject &co, geometry_msgs::PoseStamped gripper_pose, double gripper_state)
{

	double noise = 0.005;
	
	if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
		return 1;
	}
	
	double h = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
	double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
	
	//update object position based on gripperposition
	//in case die perceived position wasn't correct, it is now in the gripper
	co.primitive_poses[0].position = gripper_pose.pose.position;
	co.primitive_poses[0].position.x += Gripper::GRIPPER_DEPTH + r;
	
	//update object radius based on gripperstate
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = gripper_state/2 - noise/2;
	
	return 1;
}

int Grasping::updateGraspedBoxPose(moveit_msgs::CollisionObject &co, geometry_msgs::PoseStamped gripper_pose, double gripper_state)
{
	
	double noise = 0.005;
	
	if (co.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
		return 1;
	}
	
	double y = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
	double z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
	
	//update object position based on gripperstate
	//~ co.primitive_poses[0].position = gripper_pose.pose.position;
	//~ co.primitive_poses[0].position.z -= Gripper::GRIPPER_DEPTH + z/2 - noise;
	//~ 
	//update object size based on gripperstate
	if (y > Gripper::GRIPPER_MAX_POSITION){
		//object was grasped on the x axis
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = gripper_state - noise;
	} else {
		//object was grasped on the y axis
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = gripper_state - noise;
	}
	
	return 1;
}


template <class T>
/**
* This method formats a ros time to a string.
* Thanks to https://code.ros.org/trac/ros/ticket/2030
*/
std::string time_to_str(T ros_t)
{
  char buf[1024]      = "";
  time_t t = ros_t.sec;
  struct tm *tms = localtime(&t);
  strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
  return std::string(buf);
}

int Grasping::pick(moveit_msgs::CollisionObject co, std::string arm, 
		std::vector<geometry_msgs::PoseStamped> &poses, std::vector<geometry_msgs::PoseStamped> &pre_poses, double force)
{
	string object_name = co.id;
	move_group_interface::MoveGroup* move_group;
	if (arm == RIGHT_ARM){
		move_group = new move_group_interface::MoveGroup(*group_r_arm_);
	} else if (arm == LEFT_ARM) {
		move_group = new move_group_interface::MoveGroup(*group_l_arm_);
	} else {
		ROS_ERROR_STREAM("Grasping::pick| arm value not valide.");
		return 0;
	}

	// Goal Message to move the head
	control_msgs::PointHeadActionGoal goal_msg;

	goal_msg.header.seq = 1;
    goal_msg.header.stamp = ros::Time::now();
    // Set Goal to pre grasp position
    goal_msg.header.frame_id = pre_pose.header.frame_id;
    goal_msg.goal_id.stamp = goal_msg.header.stamp;
    // set unique id with timestamp
    goal_msg.goal_id.id = "goal_"+time_to_str(goal_msg.header.stamp);
    goal_msg.goal.target.header = goal_msg.header;
    // Set position from pre grasp
    goal_msg.goal.target.point.x = pre_pose.pose.position.x;
    goal_msg.goal.target.point.y = pre_pose.pose.position.y;
    goal_msg.goal.target.point.z = pre_pose.pose.position.z;
    goal_msg.goal.pointing_axis.x = 1;
    goal_msg.goal.pointing_axis.y = 0;
    goal_msg.goal.pointing_axis.z = 0;
    goal_msg.goal.pointing_frame = "head_plate_frame";
    goal_msg.goal.min_duration = ros::Duration(1.0);
    goal_msg.goal.max_velocity = 10;

    // Publish goal on topic /suturo/head_controller
    if( !head_publisher ) {
		ROS_INFO("Publisher invalid!\n");
	} else {
		head_publisher->publish(goal_msg);
		ROS_INFO("Published pre grasp goal: x: %f, y: %f, z: %f in Frame %s", goal_msg.goal.target.point.x,	goal_msg.goal.target.point.y, goal_msg.goal.target.point.z, goal_msg.goal.pointing_frame.c_str());
	}

	//go into pregraspposition
	int pos_id = 0;
	while (pos_id < pre_poses.size()){
		move_group->setPoseTarget(pre_poses.at(pos_id));
		ROS_INFO_STREAM("move to pregraspposition ");
		if (move_group->move()){
			break;	
		} 
		pos_id++;
	}
	
	if (pos_id == pre_poses.size()){
		ROS_ERROR_STREAM("No pregraspposition reachable for " << object_name);
		return 0;	
	}
	
	//open gripper
	double gripper_state;
	if (arm == RIGHT_ARM){
		gripper_state = gripper_->open_r_gripper();
	}else{
		gripper_state = gripper_->open_l_gripper();
	}
	
	//set goal to pose
	ROS_INFO_STREAM("set goalpose");
	move_group->setPoseTarget(poses.at(pos_id));
	
	//move Arm to goalpose
	ROS_INFO_STREAM("move to goalpose");
	if (!move_group->move()){
		ROS_ERROR_STREAM("Failed to move to " << object_name << " at: " << poses.at(pos_id));
		return 0;
	}
	
	//close grapper
	if (arm == RIGHT_ARM){
		gripper_state = gripper_->close_r_gripper(force);
	}else{
		gripper_state = gripper_->close_l_gripper(force);
	}
	
	ROS_INFO_STREAM("update objectposition in planningscene.");
	updateGraspedBoxPose(co, move_group->getCurrentPose(), gripper_state);
	updateGraspedCylinderPose(co, move_group->getCurrentPose(), gripper_state);
	
	//update collisionobject in planningscene
	if (!pi_->addObject(co)) 
		return 0;
	
	//attach object
	ROS_INFO_STREAM("attach object");
	if (!pi_->attachObject(object_name, move_group->getEndEffectorLink(), Gripper::get_r_gripper_links())) 
		return 0;
	
	//lift object by 5 cm
	//~ ROS_INFO_STREAM("lift object");
	//~ pose.pose.position.z += 0.05;
	//~ move_group->setPoseTarget(pose);
	//~ if (!move_group->move()){
		//~ ROS_INFO_STREAM("Failed to lift " << object_name);
		//~ return 0;
	//~ }
	
	return 1;
}



int Grasping::pick(std::string objectName, std::string arm, double force, ros::Publisher* head_publisher)

{
	if (!gripper_->is_connected_to_controller()){
		ROS_ERROR_STREAM("not connected to grippercontroller");
		return 0;
	}
	
	moveit_msgs::AttachedCollisionObject aco;
	moveit_msgs::CollisionObject co;
	
	if (pi_->getAttachedObject(object_name, aco))
	{
		if (arm == LEFT_ARM && aco.link_name == group_l_arm_->getEndEffectorLink() 
					|| arm == RIGHT_ARM && aco.link_name == group_r_arm_->getEndEffectorLink()){
			ROS_INFO_STREAM(object_name << " already attached.");
			return 1;
		} else {
			ROS_INFO_STREAM("Detach " << object_name << " from old arm.");
			pi_->detachObject(object_name);
			co = aco.object;
		}
	} else {
		ROS_INFO_STREAM(object_name << " not attached.");
		
		//get object from planningscene
		if (!pi_->getObject(object_name, co))
		return 0;
	}
	
	std::vector<geometry_msgs::PoseStamped> poses(0);
	std::vector<geometry_msgs::PoseStamped> pre_poses(0);
	
	
	if (!calcGraspPosition(co, poses, pre_poses))
		return 0;
	

	return pick(co, arm, pose, pre_pose, force, head_publisher);

}


int Grasping::dropObject(string object_name)
{
	if (!gripper_->is_connected_to_controller()){
		ROS_ERROR_STREAM("not connected to grippercontroller");
		return 0;
	}
	
	//get object from planningscene
	moveit_msgs::AttachedCollisionObject aco;
	if (pi_->getAttachedObject(object_name, aco))
	{
		ROS_INFO("Get object");
	} else {
		ROS_INFO_STREAM(object_name << " not attached.");
		return 1;
	}


	if(aco.link_name == group_r_arm_->getEndEffectorLink()) {
		gripper_->open_r_gripper();
	} else if (aco.link_name == group_l_arm_->getEndEffectorLink()){
		gripper_->open_l_gripper();
	}
	
	//detach object

	pi_->detachObject(object_name);
	ROS_INFO_STREAM("droped " << object_name << " successfully.");

	return 1;
}

int Grasping::drop(string arm)
{
	if (!gripper_->is_connected_to_controller()){
		ROS_ERROR_STREAM("not connected to grippercontroller");
		return 0;
	}
	
	move_group_interface::MoveGroup* move_group;
	std::string eof;
	if (arm == RIGHT_ARM){
		eof = group_r_arm_->getEndEffectorLink();
		gripper_->open_r_gripper();

	} else if (arm == LEFT_ARM) {
		eof = group_l_arm_->getEndEffectorLink();
		gripper_->open_l_gripper();

	} else {
		ROS_ERROR_STREAM("Arm value not valide.");
		return 0;
	}
	
	//get object from planningscene
	
	std::vector<moveit_msgs::AttachedCollisionObject> acos = pi_->getAttachedObjects();
	
	for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = acos.begin(); it != acos.end(); ++it){
		if (it->link_name == eof){
			pi_->detachObject(it->object.id);
		}
	}
	
	return 1;
}


int Grasping::handoff(std::string objectName, std::string new_arm, double force)
{
	//~ if (!gripper_->is_connected_to_controller()){
		//~ ROS_ERROR_STREAM("not connected to grippercontroller");
		//~ return 0;
	//~ }
//~ 
	//~ //get object from planningscene
	//~ moveit_msgs::AttachedCollisionObject aco;
	//~ if (pi_->getAttachedObject(objectName, aco))
	//~ {
		//~ ROS_INFO("Get object");
	//~ } else {
		//~ ROS_INFO_STREAM(objectName << " not attached.");
		//~ return 1;
	//~ }
	//~ 
	//~ pi_->detachObject(objectName);
//~ 
	//~ geometry_msgs::PoseStamped pose;
	//~ geometry_msgs::PoseStamped pre_pose;
	//~ 
	//~ //get object from planningscene
	//~ moveit_msgs::CollisionObject co;
	//~ if (!pi_->getObject(objectName, co))
		//~ return 0;
	//~ 
	//~ if (!calcGraspPosition(co, pose, pre_pose))
		//~ return 0;
	//~ 
	//~ bool picking = pick(co, new_arm, pose, pre_pose, force);
//~ 
	//~ if (new_arm == LEFT_ARM)
	//~ {
		//~ gripper_->open_r_gripper();
	//~ } else {
		//~ gripper_->open_l_gripper();
	//~ }
//~ 
	//~ return picking;
	return 0;
}
