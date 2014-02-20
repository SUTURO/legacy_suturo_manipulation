#include "suturo_manipulation_move_robot.h"

using namespace std;

//! ROS node initialization
MoveRobot::MoveRobot(ros::NodeHandle &nh)
{
  nh_ = nh;
  //set up the publisher for the cmd_vel topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  // localisation subscriber
  loc_sub_ = nh_.subscribe("/amcl_pose", 50, &MoveRobot::subscriberCb, this);
}

MoveRobot::~MoveRobot()
{
  
}

void MoveRobot::subscriberCb(const geometry_msgs::PoseWithCovarianceStamped& robotPoseFB){
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

bool MoveRobot::checkCollision(geometry_msgs::PoseStamped targetPose) {
  
}

//! Loop forever while sending drive commands based on keyboard input
bool MoveRobot::driveBase(geometry_msgs::PoseStamped targetPose)
{
  // std::cout << "Type a command and then press enter.  "
  //   "Use '+' to move forward, 'l' to turn left, "
  //   "'r' to turn right, '.' to exit.\n";

  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;

  while (nh_.ok() && (robotPose_.pose.position.x+0.2 > targetPose.pose.position.x || robotPose_.pose.position.x-0.2 < targetPose.pose.position.x)){
    base_cmd.linear.x = 0.5;
    cmd_vel_pub_.publish(base_cmd);
  }

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
