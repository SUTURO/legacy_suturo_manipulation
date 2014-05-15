#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <tf/transform_broadcaster.h>
#include <suturo_manipulation_mesh_loader.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

	//real
	//~ double tischposiZ = 0.81;
	//gazebo
	 //~ roslaunch pr2_teleop_general pr2_teleop_general_keyboard_bodyhead_only.launch
	double tischposiZ = 0.57;

void putObjects(ros::Publisher pub_co)
{

	
  ros::WallDuration(1.0).sleep();

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/base_footprint";

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);

  co.id = "dlink";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add box2
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05 ;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.24;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.169;
  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = tischposiZ + 0.03+ 0.09;//tischposiZ + 0.0716;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, M_PI_2, M_PI_2);
  
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

  // remove table
  co.id = "table2";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.95;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.45;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
  co.primitive_poses[0].position.x = 0.85;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = tischposiZ+0.015;
	co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);  
  pub_co.publish(co);

  // remove box1
  co.id = "cafetfilter";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add box1
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.057;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.132;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.177;
  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = 0.4;
  co.primitive_poses[0].position.z = tischposiZ + 0.03+ 0.105;//tischposiZ + 0.08;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);
  
  pub_co.publish(co);

  co.id = "corny";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add box1
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.036;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.145;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.143;
  co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = tischposiZ + 0.03 + 0.08;//tischposiZ + 0.08;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  
  // pub_co.publish(co);
  
  ros::WallDuration(2.0).sleep();
}

void muh(ros::NodeHandle &nh)
{
    ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    ros::WallDuration(1.0).sleep();
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "/base_footprint";
    co.id = "pancake_mix";
    co.operation = moveit_msgs::CollisionObject::ADD;

    Mesh_loader ml;
    co.meshes.resize(1);
    co.meshes[0] = ml.load_pancake_msg();
    co.mesh_poses.resize(1);
    co.mesh_poses[0].position.x = 0.67;
    co.mesh_poses[0].position.y = 0;
    co.mesh_poses[0].position.z = tischposiZ + 0.03 + 0.126;
    co.mesh_poses[0].orientation.w = 1;
    ros::WallDuration(1.0).sleep();
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);
    ros::WallDuration(1.0).sleep();
    co.operation = moveit_msgs::CollisionObject::ADD;
    pub_co.publish(co);
    ros::WallDuration(1.0).sleep();
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "right_arm_pick_place");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	putObjects(pub_co);
  muh(nh);
	
	ROS_INFO_STREAM("finish");
	ros::waitForShutdown();
	return 0;
}
