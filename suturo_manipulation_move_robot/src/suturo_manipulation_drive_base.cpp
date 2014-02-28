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
  ros::WallDuration(5.0).sleep();
}

Suturo_Manipulation_Move_Robot::~Suturo_Manipulation_Move_Robot(){
  
}

void Suturo_Manipulation_Move_Robot::subscriberCb(const geometry_msgs::PoseStamped& robotPoseFB){
  ROS_INFO("robotPose_ in CB: x: %f, y: %f, z: %f", robotPose_.pose.position.x, robotPose_.pose.position.y, robotPose_.pose.position.z);

  robotPose_.header.seq = robotPoseFB.header.seq;
  robotPose_.header.stamp = robotPoseFB.header.stamp;
  robotPose_.header.frame_id = robotPoseFB.header.frame_id;

  robotPose_.pose.position.x = robotPoseFB.pose.position.x;
  robotPose_.pose.position.y = robotPoseFB.pose.position.y;
  robotPose_.pose.position.z = robotPoseFB.pose.position.z;

  robotPose_.pose.orientation.w = robotPoseFB.pose.orientation.w;
  robotPose_.pose.orientation.x = robotPoseFB.pose.orientation.x;
  robotPose_.pose.orientation.y = robotPoseFB.pose.orientation.y;
  robotPose_.pose.orientation.z = robotPoseFB.pose.orientation.z;
}

bool Suturo_Manipulation_Move_Robot::checkCollision(geometry_msgs::PoseStamped targetPose) {
  // Vorschlag von Georg: Einen Kreis um den PR2 ziehen, dann eine Linie zum Ziel, dann auf der Linie prüfen, ob eine Kollision entsteht
    
  //get all collisionobjects
	std::vector<moveit_msgs::CollisionObject> cos;
	cos = pi_->getObjects();
	for (std::vector<moveit_msgs::CollisionObject>::iterator co = cos.begin(); co != cos.end(); ++co){
		if (co->primitive_poses[0].orientation.x == 0 &&
				co->primitive_poses[0].orientation.y == 0 &&
				co->primitive_poses[0].orientation.z == 0 &&
				co->primitive_poses[0].orientation.w == 1 ){
			//andere orientierung ist zu schwer ;(
			//todo? Höhe das Objekte beachten?
			//transform goalpose into objectframe

			listener_.transformPose(co->header.frame_id, targetPose, targetPose);
			
			//check dist
			if (co->primitives[0].type == shape_msgs::SolidPrimitive::BOX){
				double x_size = co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
				double y_size = co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
				double d_x = abs(co->primitive_poses[0].position.x - targetPose.pose.position.x);
				double d_y = abs(co->primitive_poses[0].position.y - targetPose.pose.position.y);
				//~ ROS_INFO_STREAM("d_x " << d_x << " d_y " << d_y);
				if (d_x <= footprint_radius + x_size/2 && d_y <= footprint_radius + y_size/2) return true;
			} 
		}
	}
	
	return false;
}

bool Suturo_Manipulation_Move_Robot::checkLocalization(){
  return (robotPose_.pose.position.x != 0 || robotPose_.pose.position.y != 0 || robotPose_.pose.position.z != 0);
}

bool Suturo_Manipulation_Move_Robot::checkXCoord(geometry_msgs::PoseStamped targetPose){
  return (robotPose_.pose.position.x > targetPose.pose.position.x+0.2 || robotPose_.pose.position.x < targetPose.pose.position.x-0.2);
}

bool Suturo_Manipulation_Move_Robot::checkYCoord(geometry_msgs::PoseStamped targetPose){
  return (robotPose_.pose.position.y > targetPose.pose.position.y+0.2 || robotPose_.pose.position.y < targetPose.pose.position.y-0.2);
}

bool Suturo_Manipulation_Move_Robot::checkOrientation(geometry_msgs::PoseStamped targetPose){
  // return (robotPose_.pose.orientation.w == targetPose.pose.orientation.w && robotPose_.pose.orientation.x == targetPose.pose.orientation.x && robotPose_.pose.orientation.y == targetPose.pose.orientation.y && robotPose_.pose.orientation.z == targetPose.pose.orientation.z);
  return true;
}

bool Suturo_Manipulation_Move_Robot::rotateBase(){
  geometry_msgs::PoseStamped rotationPose;
  // if (robotPose_.pose.orientation.w == 0 && robotPose_.pose.orientation.x == 0 && robotPose_.pose.orientation.y == 0 && robotPose_.pose.orientation.z == 0){
    rotationPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);
    
    tf::Quaternion q2(rotationPose.pose.orientation.x, rotationPose.pose.orientation.y, rotationPose.pose.orientation.z, rotationPose.pose.orientation.w);

    tf::Matrix3x3 m(q2);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("RPY rotationPose: roll: %f, pitch: %f, yaw:%f ", roll, pitch, yaw);

    // while (nh_->ok() && (robotPose_.pose.orientation.z > rotationPose.pose.orientation.z+0.1 || robotPose_.pose.orientation.z < rotationPose.pose.orientation.z-0.1) ) {
    //   base_cmd_.angular.z = 0.2;     
    //   cmd_vel_pub_.publish(base_cmd_);
    // }
    return true;
  // } else {
    // rotationPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -(M_PI_2), 0);
    // return true;
  // }

  // return false;
}


bool Suturo_Manipulation_Move_Robot::transformToBaseLink(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped poseInBaseLink){
    try{
      //transform pose to base_link
      listener_.transformPose("/base_link", pose, poseInBaseLink);
    }catch(...){
      ROS_INFO("ERROR: Transformation failed.");
      return false;
    }
  return true;
}

void Suturo_Manipulation_Move_Robot::subscriberCbLaserScan(const sensor_msgs::LaserScan& scan)
{
	for (int i = 0; i < scan.ranges.size(); i++){
		
		double alpha = scan.angle_increment*i + 0.872664626;
		double b = scan.ranges[i];
		double c = -0.275;//dist base_link to base_laser_link
		double a = sqrt((b*b) + (c*c) + (2*b*c * cos(alpha)));
		if (a < footprint_radius){
			inCollision_ = true;
			return;
		}
	}
	inCollision_ = false;
}

bool Suturo_Manipulation_Move_Robot::getInCollision()
{
	return inCollision_;
}

bool Suturo_Manipulation_Move_Robot::driveBase(geometry_msgs::PoseStamped targetPose){

	if (checkCollision(targetPose)){
		 ROS_ERROR_STREAM("targetpose in collision!");
		 return false;
	}

  // TODO: Bennys Interpolator nutzen, um bei geringerer Zielentferung eine geringere Geschwindigkeit zu nutzen
  // TODO: Falls Interpolator dass nicht macht: Wenn das x Ziel erreicht, aber y noch nicht, dann nurnoch in y Richtung starten und nicht in x etc
  // TODO: Drehen der Base um 180° einbauen
  while (!checkLocalization()){
    // Wait for localization...
  }

  geometry_msgs::PoseStamped targetPoseBaseLink;
   
  transformToBaseLink(targetPose, targetPoseBaseLink);

  ROS_INFO_STREAM(targetPose);
  ROS_INFO_STREAM(robotPose_);

  // vorwärtsfahren bis Ziel erreicht wurde
  while (nh_->ok() && checkXCoord(targetPose) && (0 < targetPoseBaseLink.pose.position.x)){

    ROS_INFO("targetPose_ in driveBase: x: %f, y: %f, z: %f", targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z);
    ROS_INFO("robotPose_ in driveBase: x: %f, y: %f, z: %f", robotPose_.pose.position.x, robotPose_.pose.position.y, robotPose_.pose.position.z);

    base_cmd_.linear.x = 0.1;
    cmd_vel_pub_.publish(base_cmd_);

    transformToBaseLink(targetPose, targetPoseBaseLink);
  }

  // seitwärtsfahren bis Ziel erreicht wurde
  while (nh_->ok() && checkYCoord(targetPose)){

    ROS_INFO("targetPose_ in driveBase: x: %f, y: %f, z: %f", targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z);
    ROS_INFO("robotPose_ in driveBase: x: %f, y: %f, z: %f", robotPose_.pose.position.x, robotPose_.pose.position.y, robotPose_.pose.position.z);

    if (0 < targetPoseBaseLink.pose.position.y){
      base_cmd_.linear.y = 0.1;
    } else {
      base_cmd_.linear.y = (-0.1);
    }
    
    cmd_vel_pub_.publish(base_cmd_);
    
    transformToBaseLink(targetPose, targetPoseBaseLink);
  }
 
  // tf::Quaternion q(robotPose_.pose.orientation.x, robotPose_.pose.orientation.y, robotPose_.pose.orientation.z, robotPose_.pose.orientation.w);

  // tf::Matrix3x3 matrix(q);
  // double roll, pitch, yaw;
  // matrix.getRPY(roll, pitch, yaw);

  // ROS_INFO("RPY robotPose_: roll: %f, pitch: %f, yaw:%f ", roll, pitch, yaw);

  // rotateBase();
  
  // while (nh_->ok() && (checkXCoord(targetPose) && checkYCoord(targetPose) && checkOrientation(targetPose))){
  //   base_cmd_.linear.x = 0.1;
  //   cmd_vel_pub_.publish(base_cmd_);
  // }

  // char cmd[50];
  // int counter = 0;
  // while(nh_.ok() ){

  // if(action == "forward"){
  //   base_cmd.linear.x = 1;
  //   base_cmd.linear.y = 1;
  //   cmd_vel_pub_.publish(base_cmd);
  //   ros::WallDuration(1).sleep();
  //   counter ++;
  // } else if (action == "rotate"){
  //   base_cmd.angular.z = -1;
  //   cmd_vel_pub_.publish(base_cmd);
  //   counter ++;
  // }
  //   std::cin.getline(cmd, 50);
  //   if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.')
  //   {
  //     std::cout << "unknown command:" << cmd << "\n";
  //     continue;
  //   }

  //   base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
  //   //move forward
  //   if(cmd[0]=='+'){
  //     base_cmd.linear.x = 0.25;
  //   } 
  //   //turn left (yaw) and drive forward at the same time
  //   else if(cmd[0]=='l'){
  //     base_cmd.angular.z = 0.75;
  //     base_cmd.linear.x = 0.25;
  //   } 
  //   //turn right (yaw) and drive forward at the same time
  //   else if(cmd[0]=='r'){
  //     base_cmd.angular.z = -0.75;
  //     base_cmd.linear.x = 0.25;
  //   } 
  //   //quit
  //   else if(cmd[0]=='.'){
  //     break;
  //   }
    
  // publish the assembled command
  // cmd_vel_pub_.publish(base_cmd);
  // }
  return true;
}

// int main(int argc, char** argv)
// {
//   //init the ROS node
//   ros::init(argc, argv, "robot_driver1");
//   ros::NodeHandle nh;

//   RobotDriver driver(nh);
//   driver.driveKeyboard(5, "forward");
// }
