/**
* This class implements the action server to grasp/drop a object.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <suturo_manipulation_msgs/suturo_manipulation_graspingAction.h>
#include <suturo_manipulation_msgs/ActionAnswer.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_grasping.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>


using namespace std;

typedef actionlib::SimpleActionServer<suturo_manipulation_msgs::suturo_manipulation_graspingAction> Server;

/**
* This method grasps or drops a object!
*/
void grop(const suturo_manipulation_msgs::suturo_manipulation_graspingGoalConstPtr& graspGoal, ros::NodeHandle* nh, Server* server_grasp)
{	
	suturo_manipulation_msgs::suturo_manipulation_graspingResult r;	
	
	r.succ.header.stamp = ros::Time();
	r.succ.type = suturo_manipulation_msgs::ActionAnswer::UNDEFINED;

	// set Planning Interface and Grasper
	ROS_INFO("Create Planning Scene Interface...");
	Suturo_Manipulation_Planning_Scene_Interface pi(nh);
	ROS_INFO("Done. Create Grasper...")	;
	Grasping grasper(&pi);
	ROS_INFO("Done.");

	// Set arm to pick and object name
	string picking_arm = graspGoal->goal.arm;
  if (picking_arm != suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM && picking_arm != suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM){
    ROS_INFO("Unknown arm! Please use suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM or suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM as names!");
    r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
    server_grasp->setAborted(r);  
  }
	ROS_INFO("Arm to pick: %s", picking_arm.c_str());
	string obj_name = graspGoal->goal.objectName;
	ROS_INFO("ObjectName to pick: %s", obj_name.c_str());

	// Check if we should grasp or drop...
	if(graspGoal->goal.grasp){
		ROS_INFO("Begin to pick object...");
		// Grasp the object
		if (grasper.pick(obj_name, picking_arm))
		{
			ROS_INFO("Object picked...");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    server_grasp->setSucceeded(r);
		} else {
			ROS_INFO("Picking failed!");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_grasp->setAborted(r);
		}
	} else {
		ROS_INFO("Begin to drop object...");
		// Drop the object
		if (grasper.drop(obj_name))
		{
			ROS_INFO("Object droped...");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::SUCCESS;
	    server_grasp->setSucceeded(r);
		} else {
			ROS_INFO("Droping failed!");
			r.succ.type = suturo_manipulation_msgs::ActionAnswer::FAIL;
		server_grasp->setAborted(r);
		}
	}
}

// create dummy data in planning scene
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

  co.id = "box2";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.07;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
  co.primitive_poses[0].position.x = 0.65;
  co.primitive_poses[0].position.y = 0.3;
  co.primitive_poses[0].position.z = 0.621;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_4);
  
  pub_co.publish(co);

  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.8;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.04;
  co.primitive_poses[0].position.x = 2;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0.5;
	co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);  
  pub_co.publish(co);

  // remove table
  co.id = "box1";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.07;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
  co.primitive_poses[0].position.x = 0.65;
  co.primitive_poses[0].position.y = -0.3;
  co.primitive_poses[0].position.z = 0.621;
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI_4);
  
  pub_co.publish(co);

  co.id = "beer2";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.03;

  co.primitive_poses[0].position.x = 0.65;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0.621;
  co.primitive_poses[0].orientation.x = 0;
  co.primitive_poses[0].orientation.y = 0;
  co.primitive_poses[0].orientation.z = 0;
  co.primitive_poses[0].orientation.w = 1;
  pub_co.publish(co);
  
  ros::WallDuration(2.0).sleep();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "suturo_manipulation_grasp_server");
	ros::NodeHandle nh;
	
	// Publishing the planning scene
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	putObjects(pub_co);
	
	Server server_grasp(nh, "suturo_man_grasping_server", boost::bind(&grop, _1, &nh, &server_grasp), false);
	server_grasp.start();
	

	ROS_INFO("Ready to grasp!!!");

	ros::spin();
	return 0;
}
