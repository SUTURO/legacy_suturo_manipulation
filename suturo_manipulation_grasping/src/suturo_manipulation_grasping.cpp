#include "suturo_manipulation_grasping.h"

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
	
	//wait because ros
	ros::WallDuration(0.5).sleep();
}

Grasping::~Grasping()
{

}

void Grasping::publishTfFrame(moveit_msgs::CollisionObject co)
{

  ROS_INFO_STREAM("publish object frame " << co.id);
  
  transform_.setOrigin( tf::Vector3(co.primitive_poses[0].position.x, 
				co.primitive_poses[0].position.y, co.primitive_poses[0].position.z) );
				
  transform_.setRotation( tf::Quaternion(co.primitive_poses[0].orientation.x,
					co.primitive_poses[0].orientation.y,
					co.primitive_poses[0].orientation.z,
					co.primitive_poses[0].orientation.w) );
					
  br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "odom_combined", co.id));
}


void Grasping::addBoxGraspPositionsZ(double z, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	
	pose.header.frame_id = frame_id;
	pre_pose.header.frame_id = frame_id;
	
	//grasp from above
	pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, rotation);

	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = Gripper::GRIPPER_DEPTH + z/2;
	pre_pose = pose;
	pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH -0.05;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);
	

	//grasp from below
	pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, rotation);
		
	pose.pose.position.z = 0 - Gripper::GRIPPER_DEPTH - z/2;
	pre_pose = pose;
	pre_pose.pose.position.z -= Gripper::GRIPPER_DEPTH -0.05;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);	
}				

void Grasping::addBoxGraspPositionsY(double y, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	
	pose.header.frame_id = frame_id;
	pre_pose.header.frame_id = frame_id;
	
	//links
	pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, -M_PI_2);
	
	pose.pose.position.x = 0;
	pose.pose.position.y = Gripper::GRIPPER_DEPTH + y/2;
	pose.pose.position.z = 0;
	pre_pose = pose;
	pre_pose.pose.position.y += Gripper::GRIPPER_DEPTH -0.05;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);
	
	//rechts		
	
	pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI_2);	
	pose.pose.position.y = 0 - Gripper::GRIPPER_DEPTH - y/2;
	pre_pose = pose;
	pre_pose.pose.position.y -= Gripper::GRIPPER_DEPTH -0.05;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);
}				

void Grasping::addBoxGraspPositionsX(double x, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pre_pose;
	
	pose.header.frame_id = frame_id;
	pre_pose.header.frame_id = frame_id;
	
	//posi 1
	pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI);
	
	pose.pose.position.x = Gripper::GRIPPER_DEPTH + x/2;
	pose.pose.position.y = 0;
	pose.pose.position.z = 0;
	pre_pose = pose;
	pre_pose.pose.position.x += Gripper::GRIPPER_DEPTH -0.05;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);
	
	//posi 2
	pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, 0);	
	pose.pose.position.x = 0 - Gripper::GRIPPER_DEPTH - x/2;
	pre_pose = pose;
	pre_pose.pose.position.x -= Gripper::GRIPPER_DEPTH -0.05;
	
	poses.push_back(pose);
	pre_poses.push_back(pre_pose);
}				

int Grasping::calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses, 
				std::vector<geometry_msgs::PoseStamped> &pre_poses)
{
	ROS_INFO_STREAM("calculate graspposition for " << co.id);
	
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
			y > Gripper::GRIPPER_MAX_POSITION &&
			z > Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("Object is to big!");
		return 0;
	}
	
	publishTfFrame(co);
	int result = 1;
	if (x < Gripper::GRIPPER_MAX_POSITION){
		ROS_ERROR_STREAM("grasping x");
		addBoxGraspPositionsZ(z, M_PI_2, co.id, poses, pre_poses);
	
		addBoxGraspPositionsY(y, 0, co.id, poses, pre_poses);
		result *= 2;
	} 
	if (y < Gripper::GRIPPER_MAX_POSITION) {
ROS_ERROR_STREAM("grasping y");
		addBoxGraspPositionsZ(z, 0, co.id, poses, pre_poses);
		
		addBoxGraspPositionsX(x, 0, co.id, poses, pre_poses);
		result *= 3;
	}
	if (z < Gripper::GRIPPER_MAX_POSITION) {
ROS_ERROR_STREAM("grasping z");
		addBoxGraspPositionsX(x, M_PI_2, co.id, poses, pre_poses);
		
		addBoxGraspPositionsY(y, M_PI_2, co.id, poses, pre_poses);
		result *= 5;
	}

	return result;
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

int Grasping::updateGraspedBoxPose(moveit_msgs::CollisionObject &co, int graspable_sides, int pos_id, double gripper_state)
{
	
	double noise = 0.005;
	
	if (co.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
		return 1;
	}
  
	//update object size based on gripperstate
	//ultra hacky shit :(
	
	if (graspable_sides % 2 == 0 && pos_id <= 3){
		//x axis has been grasped
		ROS_INFO_STREAM("x axis has been grasped");
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = gripper_state - noise;
	} else if (graspable_sides % 3 == 0 && (pos_id <= 3 || graspable_sides % 2 == 0 && (pos_id >= 4	&& pos_id <= 7))){
		//y axis has been grasped
		ROS_INFO_STREAM("y axis has been grasped");
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = gripper_state - noise;
	} else {
		//z axis has been grasped
		ROS_INFO_STREAM("z axis has been grasped");
		co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = gripper_state - noise;
	}
	
	return 1;
}

int Grasping::pick(moveit_msgs::CollisionObject co, std::string arm, 
		std::vector<geometry_msgs::PoseStamped> &poses, std::vector<geometry_msgs::PoseStamped> &pre_poses, 
		double force, int graspable_sides)
{
	string object_name = co.id;
	
	//select movegroup depending on arm
	move_group_interface::MoveGroup* move_group;
	if (arm == RIGHT_ARM){
		move_group = new move_group_interface::MoveGroup(*group_r_arm_);
	} else if (arm == LEFT_ARM) {
		move_group = new move_group_interface::MoveGroup(*group_l_arm_);
	} else {
		ROS_ERROR_STREAM("Arm value not valide.");
		return 0;
	}
	
	//search for valid pregraspposition and move there
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
	
	//close gripper
	if (arm == RIGHT_ARM){
		gripper_state = gripper_->close_r_gripper(force);
	}else{
		gripper_state = gripper_->close_l_gripper(force);
	}
	
	ROS_INFO_STREAM("update objectposition in planningscene.");
	updateGraspedBoxPose(co, graspable_sides, pos_id, gripper_state);
	updateGraspedCylinderPose(co, move_group->getCurrentPose(), gripper_state);
	
	//update collisionobject in planningscene
	if (!pi_->addObject(co)) 
		return 0;
	
	//attach object
	ROS_INFO_STREAM("attach object");
	if (!pi_->attachObject(object_name, move_group->getEndEffectorLink(), Gripper::get_r_gripper_links())) 
		return 0;
	
	return 1;
}


int Grasping::pick(std::string object_name, std::string arm, double force)
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
	
	int graspable_sides = calcGraspPosition(co, poses, pre_poses);
	if (graspable_sides == 0)
		return 0;
	
	return pick(co, arm, poses, pre_poses, force, graspable_sides);
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
