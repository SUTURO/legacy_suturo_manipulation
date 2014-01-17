#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_grasping.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_planning_scene_interface.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void putObjects(ros::Publisher pub_co)
{
	//real
	//~ double tischposiZ = 0.86;
	//gazebo
	 //~ roslaunch pr2_teleop_general pr2_teleop_general_keyboard_bodyhead_only.launch
	double tischposiZ = 0.5;
	
  ros::WallDuration(1.0).sleep();

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/base_footprint";

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);

  co.id = "box2";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add box2
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.145;
  //~ co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.01;
  //~ co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.16;
  //~ co.primitive_poses[0].position.x = 0.652;
  //~ co.primitive_poses[0].position.y = 0;
  //~ co.primitive_poses[0].position.z = 0.941;
  //~ co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  //~ ROS_INFO_STREAM(co.primitive_poses[0].position);
  //~ pub_co.publish(co);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.045;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.16;
  co.primitive_poses[0].position.x = 0.652;
  co.primitive_poses[0].position.y = 0.3;
  co.primitive_poses[0].position.z = tischposiZ + 0.08;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_4);
  
  pub_co.publish(co);

  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.75;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.8;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tischposiZ;
  co.primitive_poses[0].position.x = 0.85;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = tischposiZ/2;
	co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);  
  pub_co.publish(co);

  // remove box1
  co.id = "box1";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add box1
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.045;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.145;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.16;
  co.primitive_poses[0].position.x = 0.65;
  co.primitive_poses[0].position.y = -0.3;
  co.primitive_poses[0].position.z = tischposiZ + 0.08;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_4);
  
  pub_co.publish(co);

  co.id = "beer2";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.25;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.0375;

  co.primitive_poses[0].position.x = 0.65;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = tischposiZ + 0.125;
  co.primitive_poses[0].orientation.x = 0;
  co.primitive_poses[0].orientation.y = 0;
  co.primitive_poses[0].orientation.z = 0;
  co.primitive_poses[0].orientation.w = 1;
  pub_co.publish(co);

  ros::WallDuration(2.0).sleep();
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "right_arm_pick_place");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	putObjects(pub_co);
	
	ROS_INFO_STREAM("finish");
	ros::waitForShutdown();
	return 0;
}
