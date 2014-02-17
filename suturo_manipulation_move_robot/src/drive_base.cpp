// #include <iostream>

// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <tf/transform_listener.h>

// class RobotDriver
// {
// private:
//   //! The node handle we'll be using
//   ros::NodeHandle nh_;
//   //! We will be publishing to the "cmd_vel" topic to issue commands
//   ros::Publisher cmd_vel_pub_;
//   //! We will be listening to TF transforms as well
//   tf::TransformListener listener_;

// public:
//   //! ROS node initialization
//   RobotDriver(ros::NodeHandle &nh)
//   {
//     nh_ = nh;
//     //set up the publisher for the cmd_vel topic
//     cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
//   }

//   //! Drive forward a specified distance based on odometry information
//   bool driveForwardOdom(double distance)
//   {
//     ROS_INFO("1");
//     //wait for the listener to get the first message
//     listener_.waitForTransform("base_footprint", "odom_combined", 
//                                ros::Time(0), ros::Duration(100.0));
    
//     //we will record transforms here
//     tf::StampedTransform start_transform;
//     tf::StampedTransform current_transform;

//     ROS_INFO("2");
//     //record the starting transform from the odometry to the base frame
//     listener_.lookupTransform("base_footprint", "odom_combined", 
//                               ros::Time(0), start_transform);
    
//     //we will be sending commands of type "twist"
//     geometry_msgs::Twist base_cmd;
//     //the command will be to go forward at 0.25 m/s
//     base_cmd.linear.y = base_cmd.angular.z = 0;
//     base_cmd.linear.x = 0.25;
    
//     ros::Rate rate(10.0);
//     bool done = false;
//     while (!done && nh_.ok())
//     {
//       //send the drive command
//       cmd_vel_pub_.publish(base_cmd);
//       rate.sleep();
//       //get the current transform
//       // try
//       // {
//         ROS_INFO("3");
//         listener_.lookupTransform("base_footprint", "odom_combined", 
//                                   ros::Time(0), current_transform);
//       // }
//       // catch (tf::TransformException ex)
//       // {
//       //   ROS_ERROR("%s",ex.what());
//       //   break;
//       // }
//       //see how far we've traveled
//       tf::Transform relative_transform = 
//         start_transform.inverse() * current_transform;
//       double dist_moved = relative_transform.getOrigin().length();

//       if(dist_moved > distance) done = true;
//     }
//     if (done) return true;
//     return false;
//   }
// };

// int main(int argc, char** argv)
// {
//   //init the ROS node
//   ros::init(argc, argv, "robot_driver");
//   ros::NodeHandle nh;

//   RobotDriver driver(nh);
//   driver.driveForwardOdom(0.5);
// }

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  }

  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard(double forward, string action)
  {
    // std::cout << "Type a command and then press enter.  "
    //   "Use '+' to move forward, 'l' to turn left, "
    //   "'r' to turn right, '.' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    char cmd[50];
    int counter = 0;
    while(nh_.ok() ){

    if(action == "forward"){
      base_cmd.linear.x = 1;
      base_cmd.linear.y = 1;
      cmd_vel_pub_.publish(base_cmd);
      ros::WallDuration(1).sleep();
      counter ++;
    } else if (action == "rotate"){
      base_cmd.angular.z = -1;
      cmd_vel_pub_.publish(base_cmd);
      counter ++;
    }
      // std::cin.getline(cmd, 50);
      // if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.')
      // {
      //   std::cout << "unknown command:" << cmd << "\n";
      //   continue;
      // }

      // base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      // //move forward
      // if(cmd[0]=='+'){
      //   base_cmd.linear.x = 0.25;
      // } 
      // //turn left (yaw) and drive forward at the same time
      // else if(cmd[0]=='l'){
      //   base_cmd.angular.z = 0.75;
      //   base_cmd.linear.x = 0.25;
      // } 
      // //turn right (yaw) and drive forward at the same time
      // else if(cmd[0]=='r'){
      //   base_cmd.angular.z = -0.75;
      //   base_cmd.linear.x = 0.25;
      // } 
      // //quit
      // else if(cmd[0]=='.'){
      //   break;
      // }
      
    // publish the assembled command
    // cmd_vel_pub_.publish(base_cmd);
    }
    return true;
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver1");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveKeyboard(5, "forward");
}
