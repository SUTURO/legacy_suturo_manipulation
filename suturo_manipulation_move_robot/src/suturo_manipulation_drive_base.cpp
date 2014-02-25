#include "suturo_manipulation_move_robot.h"

using namespace std;

//! ROS node initialization
Suturo_Manipulation_Move_Robot::Suturo_Manipulation_Move_Robot(ros::NodeHandle* nodehandle){
  nh_ = nodehandle;
  //set up the publisher for the cmd_vel topic
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  // localisation subscriber
  loc_sub_ = nh_->subscribe("/amcl_pose", 50, &Suturo_Manipulation_Move_Robot::subscriberCb, this);
  ros::WallDuration(5.0).sleep();
}

Suturo_Manipulation_Move_Robot::~Suturo_Manipulation_Move_Robot(){
  
}

void Suturo_Manipulation_Move_Robot::subscriberCb(const geometry_msgs::PoseWithCovarianceStamped& robotPoseFB){
  ROS_INFO("robotPose_ in CB: x: %f, y: %f, z: %f", robotPose_.pose.position.x, robotPose_.pose.position.y, robotPose_.pose.position.z);

  robotPose_.header.seq = robotPoseFB.header.seq;
  robotPose_.header.stamp = robotPoseFB.header.stamp;
  robotPose_.header.frame_id = robotPoseFB.header.frame_id;

  robotPose_.pose.position.x = robotPoseFB.pose.pose.position.x;
  robotPose_.pose.position.y = robotPoseFB.pose.pose.position.y;
  robotPose_.pose.position.z = robotPoseFB.pose.pose.position.z;

  robotPose_.pose.orientation.w = robotPoseFB.pose.pose.orientation.w;
  robotPose_.pose.orientation.x = robotPoseFB.pose.pose.orientation.x;
  robotPose_.pose.orientation.y = robotPoseFB.pose.pose.orientation.y;
  robotPose_.pose.orientation.z = robotPoseFB.pose.pose.orientation.z;
}

bool Suturo_Manipulation_Move_Robot::checkCollision(geometry_msgs::PoseStamped targetPose) {
  // Vorschlag von Georg: Einen Kreis um den PR2 ziehen, dann eine Linie zum Ziel, dann auf der Linie prüfen, ob eine Kollision entsteht
}

bool Suturo_Manipulation_Move_Robot::checkXCoord(geometry_msgs::PoseStamped targetPose){
  return (robotPose_.pose.position.x+0.2 > targetPose.pose.position.x || robotPose_.pose.position.x-0.2 < targetPose.pose.position.x);
}

bool Suturo_Manipulation_Move_Robot::checkYCoord(geometry_msgs::PoseStamped targetPose){
  return (robotPose_.pose.position.y+0.2 > targetPose.pose.position.y || robotPose_.pose.position.y-0.2 < targetPose.pose.position.y);
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

bool Suturo_Manipulation_Move_Robot::driveBase(geometry_msgs::PoseStamped targetPose){

  // TODO: Bennys Interpolator nutzen, um bei geringerer Zielentferung eine geringere Geschwindigkeit zu nutzen
  // TODO: Falls Interpolator dass nicht macht: Wenn das x Ziel erreicht, aber y noch nicht, dann nurnoch in y Richtung starten und nicht in x etc
  // TODO: Drehen der Base um 180° einbauen
  // TODO: Abfangen von 0 0 0 wenn amcl keine Pose gepublished hat

  // vorwärtsfahren bis Ziel erreicht wurde
  //while (nh_->ok() && checkXCoord(targetPose)){
  while(nh_->ok()){  
    ROS_INFO("robotPose_ in driveBase: x: %f, y: %f, z: %f", robotPose_.pose.position.x, robotPose_.pose.position.y, robotPose_.pose.position.z);
    //base_cmd_.linear.x = 0.1;
    //cmd_vel_pub_.publish(base_cmd_);
  }

  // seitwärtsfahren bis Ziel erreicht wurde
  //while (nh_->ok() && checkYCoord(targetPose)){
   // base_cmd_.linear.y = 0.1;
   // cmd_vel_pub_.publish(base_cmd_);
 // }
 
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
