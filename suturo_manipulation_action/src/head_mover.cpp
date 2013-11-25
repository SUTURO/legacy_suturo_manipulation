/* Author: Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <sensor_msgs/PointCloud2.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

int senden;
ros::NodeHandle *nh = NULL;
ros::Publisher *pub = NULL;

void umleiter(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
	
	if (senden > 0){
		ROS_INFO_STREAM("asdasdasd-----------------------" << senden);
		pub->publish(*inputCloud);
		//senden--;	
	}
}

int pub_new_scene()
{
	ros::Subscriber sub = nh->subscribe("head_mount_kinect/depth_registered/points", 1000, &umleiter);
}


int main(int argc, char **argv)
{
  senden = 1;
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  nh = new (ros::NodeHandle);
  ros::WallDuration sleep_time(10.0);
  pub = new (ros::Publisher);
  *pub = nh->advertise<sensor_msgs::PointCloud2>("blubb", 1);
  ros::Subscriber sub = nh->subscribe("head_mount_kinect/depth_registered/points", 1000, &umleiter);
    ros::WallDuration(1.0).sleep();
    senden = true;
/*  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("right_arm");
  group.setPlanningTime(20.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "odom_combined";

  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);



  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.175;
  pub_co.publish(co);



  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 0.7;
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();
//------------------------------
  */
  
  

  
  ros::Publisher planning_scene_diff_publisher = nh->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
  
  moveit_msgs::PlanningScene planning_scene;
  
  
  ROS_INFO("1");
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "odom_combined";
  co.id = "part";
  //co.operation = moveit_msgs::CollisionObject::REMOVE;
 // planning_scene.world.collision_objects.push_back(co);
 // planning_scene.is_diff = true;
 //  planning_scene_diff_publisher.publish(planning_scene); 
  
ROS_INFO("2");
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);  
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.4;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
ROS_INFO("3");
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 0.7;  
ROS_INFO("4");


  /* PUT THE OBJECT IN THE ENVIRONMENT*/
  ROS_INFO("Putting the object back into the environment");

  planning_scene.world.collision_objects.push_back(co);
  
  
	/*planning_scene.allowed_collision_matrix.entry_names.resize(1);
	planning_scene.allowed_collision_matrix.entry_names[0] = "part";
	planning_scene.allowed_collision_matrix.entry_values.resize(1);
	planning_scene.allowed_collision_matrix.entry_values[0].enabled.resize(1);
	planning_scene.allowed_collision_matrix.entry_values[0].enabled[0] = true;

	
	planning_scene.allowed_collision_matrix.default_entry_names.resize(1);
	planning_scene.allowed_collision_matrix.default_entry_names[0] = "part";
	planning_scene.allowed_collision_matrix.default_entry_values.resize(1);
	planning_scene.allowed_collision_matrix.default_entry_values[0] = true;
  */
	planning_scene.is_diff = true;  
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_time.sleep();
senden = -1;
co.operation = moveit_msgs::CollisionObject::REMOVE;
 planning_scene.world.collision_objects.clear();
 planning_scene.world.collision_objects.push_back(co);

planning_scene_diff_publisher.publish(planning_scene);
sleep_time.sleep();

  moveit::planning_interface::MoveGroup group("right_arm");
  group.setPlanningTime(20.0);
  group.setPositionTarget(0.7, 0.0, 0.7);
  group.move();
senden = 1;

  ros::WallDuration(1.0).sleep();


  ros::waitForShutdown();
  return 0;
}





/*#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_moveAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>

using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_moveAction> Server;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "suturo_manipulation_move_head");
  ros::NodeHandle n;
  
  if (argc != 3)
  {
    ROS_INFO("usage: tilt pitch");
    return 1;
  }
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  move_group_interface::MoveGroup group("head");
  vector<double> p;
  p.push_back(atof(argv[1]));
  p.push_back(atof(argv[2]));
  group.setJointValueTarget(p);
  group.move();

  ros::waitForShutdown();
  return 0;
}*/
