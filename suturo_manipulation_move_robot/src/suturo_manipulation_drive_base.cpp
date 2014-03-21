#include "suturo_manipulation_move_robot.h"

using namespace std;

//! ROS node initialization
Suturo_Manipulation_Move_Robot::Suturo_Manipulation_Move_Robot(ros::NodeHandle* nodehandle){
  nh_ = nodehandle;
  
  pi_ = new Suturo_Manipulation_Planning_Scene_Interface(nh_);
  
  //set up the publisher for the cmd_vel topic
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  // localisation subscriber
  loc_sub_ = nh_->subscribe("/suturo/robot_location", 50, &Suturo_Manipulation_Move_Robot::subscriberCb, this);
  collision_sub_ = nh_->subscribe("/base_scan", 50, &Suturo_Manipulation_Move_Robot::subscriberCbLaserScan, this);
  ros::WallDuration(1.0).sleep();
}

Suturo_Manipulation_Move_Robot::~Suturo_Manipulation_Move_Robot(){
  
}

void Suturo_Manipulation_Move_Robot::subscriberCb(const geometry_msgs::PoseStamped& robotPoseFB){
  robotPose_.header = robotPoseFB.header;
  robotPose_.pose = robotPoseFB.pose;
}

bool Suturo_Manipulation_Move_Robot::checkFullCollision(double danger_zone)
{
	moveit_msgs::PlanningScene ps;
	pi_->getPlanningScene(ps);
	
	//increase collisionobject size
	for (int i = 0; i < ps.world.collision_objects.size(); i++){
		moveit_msgs::CollisionObject &co = ps.world.collision_objects[i];
		if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX){
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] += danger_zone;
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] += danger_zone;
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] += danger_zone;
		} else if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER){
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] += danger_zone;
			co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] += danger_zone;
		}
	}

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
	
	planning_scene.setPlanningSceneMsg(ps);
	
	collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
	
	collision_result.clear();
	planning_scene.checkCollision(collision_request, collision_result);
	
	
	return collision_result.collision;
}

bool Suturo_Manipulation_Move_Robot::checkCollision(geometry_msgs::PoseStamped targetPose) {
  //get all collisionobjects
	std::vector<moveit_msgs::CollisionObject> cos;
	
	if (!pi_->getObjects(cos)) return true;
	for (std::vector<moveit_msgs::CollisionObject>::iterator co = cos.begin(); co != cos.end(); ++co){
		if (co->primitive_poses[0].orientation.x == 0 &&
				co->primitive_poses[0].orientation.y == 0 &&
				co->primitive_poses[0].orientation.z == 0 &&
				co->primitive_poses[0].orientation.w == 1 ){
			//andere orientierung ist zu schwer ;(
			//aber alle gegenstände von knowledge werden mit dieser orientierung gepublisht
			//todo? Höhe das Objekte beachten?
			
			//transform goalpose into objectframe
			listener_.transformPose(co->header.frame_id, targetPose, targetPose);
			
			//check dist
			if (co->primitives[0].type == shape_msgs::SolidPrimitive::BOX){
				double x_size = co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
				double y_size = co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
				double d_x = abs(co->primitive_poses[0].position.x - targetPose.pose.position.x);
				double d_y = abs(co->primitive_poses[0].position.y - targetPose.pose.position.y);
				if (d_x <= footprint_radius + x_size/2 && d_y <= footprint_radius + y_size/2) return true;
			} 
		}
	}
	
	return false;
}

bool Suturo_Manipulation_Move_Robot::checkLocalization(){
  return (robotPose_.pose.position.x != 0 || robotPose_.pose.position.y != 0 || robotPose_.pose.position.z != 0);
}

bool Suturo_Manipulation_Move_Robot::xCoordArrived(geometry_msgs::PoseStamped targetPose){
  // return !(robotPose_.pose.position.x > targetPose.pose.position.x+0.01 || robotPose_.pose.position.x < targetPose.pose.position.x-0.01);
  return (robotPose_.pose.position.x < targetPose.pose.position.x+0.01 && robotPose_.pose.position.x > targetPose.pose.position.x-0.01);
}

bool Suturo_Manipulation_Move_Robot::yCoordArrived(geometry_msgs::PoseStamped targetPose){
  // return !(robotPose_.pose.position.y > targetPose.pose.position.y+0.01 || robotPose_.pose.position.y < targetPose.pose.position.y-0.01);
  return (robotPose_.pose.position.y < targetPose.pose.position.y+0.01 && robotPose_.pose.position.y > targetPose.pose.position.y-0.01);
}

bool Suturo_Manipulation_Move_Robot::orientationArrived(tf::Quaternion robotOrientation, tf::Quaternion* targetOrientation){
  return (targetOrientation->angle(robotOrientation) < 0.05) && (targetOrientation->angle(robotOrientation) > -0.05);
  // return !((robotOrientation.angle(&targetOrientation) > 0.01) || (robotOrientation.angle(&targetOrientation) < -0.01));
}

bool Suturo_Manipulation_Move_Robot::calculateYTwist(tf::Quaternion* targetQuaternion){
  // TODO: Schöner machen

  yTwist_ = 0;

  geometry_msgs::PoseStamped homePose;
  geometry_msgs::PoseStamped homePose180;

  geometry_msgs::PoseStamped homePoseBase;
  geometry_msgs::PoseStamped homePose180Base;

  homePose.header.frame_id = "/map";
  homePose180.header.frame_id = "/map";

  homePose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
  homePose180.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  transformToBaseLink(homePose, homePoseBase);
  transformToBaseLink(homePose180, homePose180Base);

  tf::Quaternion robotPoseQuaternion(0,0,0,1);
  tf::Quaternion homePoseOuaternion(homePoseBase.pose.orientation.x, homePoseBase.pose.orientation.y, homePoseBase.pose.orientation.z, homePoseBase.pose.orientation.w);
  tf::Quaternion homePose180Quaternion(homePose180Base.pose.orientation.x, homePose180Base.pose.orientation.y, homePose180Base.pose.orientation.z, homePose180Base.pose.orientation.w);

  robotToHome_ = robotPoseQuaternion.angle(homePoseOuaternion);
  robotToHome180_ = robotPoseQuaternion.angle(homePose180Quaternion);

  if ( robotToHome_ < robotToHome180_) {
    yTwist_ = 0.2;
    return true;
  } else {
    yTwist_ = -0.2;
    return true;
  }
  return false;
}


bool Suturo_Manipulation_Move_Robot::rotateBase(){

  tf::Quaternion *targetQuaternion = new tf::Quaternion(targetPoseBaseLink_.pose.orientation.x, targetPoseBaseLink_.pose.orientation.y, targetPoseBaseLink_.pose.orientation.z, targetPoseBaseLink_.pose.orientation.w);
  tf::Quaternion robotOrientation(0, 0, 0, 1);

  ROS_INFO("Begin to rotate base");

  if (calculateYTwist(targetQuaternion)){
		//TODO:HEILE MACHEN
    //~ while (nh_->ok() && !orientationArrived(robotOrientation, targetQuaternion) && !getInCollision() && transformToBaseLink(targetPose_, targetPoseBaseLink_)) {
    while (nh_->ok() && !orientationArrived(robotOrientation, targetQuaternion) && transformToBaseLink(targetPose_, targetPoseBaseLink_)) {
      base_cmd_.angular.z = yTwist_;
      cmd_vel_pub_.publish(base_cmd_);

      targetQuaternion = new tf::Quaternion(targetPoseBaseLink_.pose.orientation.x, targetPoseBaseLink_.pose.orientation.y, targetPoseBaseLink_.pose.orientation.z, targetPoseBaseLink_.pose.orientation.w);
      ROS_INFO_STREAM(targetQuaternion->angle(robotOrientation));
    }
  }
  ROS_INFO("rotateBase done");
  return true;
}


bool Suturo_Manipulation_Move_Robot::transformToBaseLink(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink){
  try{
    //transform pose to base_link
    listener_.transformPose("/base_link", pose, poseInBaseLink);
  }catch(...){
    ROS_INFO("ERROR: Transformation failed.");
    return false;
  }
  return true;
}

void Suturo_Manipulation_Move_Robot::subscriberCbLaserScan(const sensor_msgs::LaserScan& scan){
	collisions_.clear();
	for (int i = 0; i < scan.ranges.size(); i++){
		
		//cosinussatz um abstand zu mittelpunkt zu berechnen
		double alpha = scan.angle_increment*i + 0.872664626;
		double b = scan.ranges[i];
		double c = -0.275;//dist base_link to base_laser_link
		double a = sqrt((b*b) + (c*c) + (2*b*c * cos(alpha)));
		if (a < footprint_radius){
			collisions_.push_back(alpha);
		}
	}
	if (!collisions_.empty()){
		ROS_ERROR_STREAM("COLLISION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!111elf" );
	}
}

std::vector<double> Suturo_Manipulation_Move_Robot::getCollisions(){
	return collisions_;
}

bool Suturo_Manipulation_Move_Robot::driveBase(geometry_msgs::PoseStamped targetPose){

  // TODO: Bennys Interpolator nutzen, um bei geringerer Zielentferung eine geringere Geschwindigkeit zu nutzen
  // TODO: Falls Interpolator dass nicht macht: Wenn das x Ziel erreicht, aber y noch nicht, dann nurnoch in y Richtung starten und nicht in x etc
  
  base_cmd_.linear.x = 0;
  base_cmd_.linear.y = 0;
  base_cmd_.angular.z = 0;

  targetPose_ = targetPose;

	if (checkCollision(targetPose_)){
		 ROS_ERROR_STREAM("targetpose in collision!");
		 return false;
	}

  while (!checkLocalization()){
    // Wait for localization...
  }
   
  transformToBaseLink(targetPose_, targetPoseBaseLink_);
  ROS_INFO("transform drivebase");

  // rotateBase();    
  base_cmd_.angular.z = 0;
  
  ROS_INFO("begin to move vorward");
  // move vorward
  //TODO: HEILE MACHEN
  //~ while (nh_->ok() && !xCoordArrived(targetPose) && !getInCollision() && targetPoseBaseLink_.pose.position.x > 0){
  while (nh_->ok() && !xCoordArrived(targetPose) && targetPoseBaseLink_.pose.position.x > 0){
    base_cmd_.linear.x = 0.1;
    cmd_vel_pub_.publish(base_cmd_);

    transformToBaseLink(targetPose_, targetPoseBaseLink_);
  }

  ROS_INFO("move forward done, begin to move sideward");
  bool check = false;
  // move sideward
  //TODO: HEILE MACHEN
  //~ while (nh_->ok() && !yCoordArrived(targetPose_) && !getInCollision()){
  while (nh_->ok() && !yCoordArrived(targetPose_)){

    base_cmd_.linear.x = 0;

    // check if goal is on the left or right side
    if (0 < targetPoseBaseLink_.pose.position.y && !check){
      base_cmd_.linear.y = 0.1;
      check = true;
    } else {
      base_cmd_.linear.y = (-0.1);
      check = true;
    }
    
    cmd_vel_pub_.publish(base_cmd_);
    
    transformToBaseLink(targetPose_, targetPoseBaseLink_);
    std::cout << "Transformed pose:" << targetPoseBaseLink_.pose.position.x << " ";
    std::cout << targetPoseBaseLink_.pose.position.y << std::endl;
  }
  ROS_INFO("move sideward done, target should be arrived");

  ROS_INFO_STREAM(targetPose_);
  ROS_INFO_STREAM(robotPose_);

  return true;   
}
