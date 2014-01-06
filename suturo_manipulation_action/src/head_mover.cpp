#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_grasping.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_planning_scene_interface.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(moveit::planning_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.585;
  p.pose.position.y = 0;
  p.pose.position.z = 0.625;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  manipulation_msgs::Grasp g;
  g.grasp_pose = p;

  g.approach.direction.vector.x = 1.0;
  g.approach.direction.header.frame_id = "r_wrist_roll_link";
  g.approach.min_distance = 0.2;
  g.approach.desired_distance = 0.4;

  g.retreat.direction.header.frame_id = "base_footprint";
  g.retreat.direction.vector.z = 1.0;
  g.retreat.min_distance = 0.1;
  g.retreat.desired_distance = 0.25;

  g.pre_grasp_posture.name.resize(1, "r_gripper_l_finger_joint");
  g.pre_grasp_posture.position.resize(1);
  g.pre_grasp_posture.position[0] = 0.55;
  g.grasp_posture.name.resize(1, "r_gripper_l_finger_joint");
  g.grasp_posture.position.resize(1);
  g.grasp_posture.position[0] = 0;

  grasps.push_back(g);
 // group.setSupportSurfaceName("table");
  group.pick("part", grasps);
}

void place(moveit::planning_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::PlaceLocation> loc;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.7;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  manipulation_msgs::PlaceLocation g;
  g.place_pose = p;

  g.approach.direction.vector.z = -1.0;
  g.retreat.direction.vector.x = -1.0;
  g.retreat.direction.header.frame_id = "base_footprint";
  g.approach.direction.header.frame_id = "r_wrist_roll_link";
  g.approach.min_distance = 0.1;
  g.approach.desired_distance = 0.2;
  g.retreat.min_distance = 0.1;
  g.retreat.desired_distance = 0.25;

  g.post_place_posture.name.resize(1, "r_gripper_l_finger_joint");
  g.post_place_posture.position.resize(1);
  g.post_place_posture.position[0] = 0.548;

  loc.push_back(g);
  group.setSupportSurfaceName("table");


  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  group.setPathConstraints(constr);
  group.setPlannerId("RRTConnectkConfigDefault");

  group.place("part", loc);
}

void putObjects(ros::Publisher pub_co)
{
  ros::WallDuration(1.0).sleep();

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "/base_footprint";

  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  //pub_co.publish(co);

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
 // pub_co.publish(co);



  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1;
  co.primitive_poses[0].position.x = 1.5;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0.01;
 // pub_co.publish(co);



  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.25;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.03;

  co.primitive_poses[0].position.x = 0.76;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0.635;
  co.primitive_poses[0].orientation.x = 0.5;
  co.primitive_poses[0].orientation.y = 0.5;
  co.primitive_poses[0].orientation.z = 0.5;
  co.primitive_poses[0].orientation.w = 0.5;
  
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(co.primitive_poses[0].orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
  co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, M_PI_4, M_PI_4);
  ROS_INFO_STREAM(co.primitive_poses[0].orientation);
  pub_co.publish(co);
  
  tf::quaternionMsgToTF(co.primitive_poses[0].orientation, q);
tf::Vector3 vector(0, 0, 1);
tf::Vector3 rotated_vector = tf::quatRotate(q, vector);
  ROS_INFO_STREAM("v3 " << vector << " rotated " << rotated_vector);
  // wait a bit for ros things to initialize
  ros::WallDuration(2.0).sleep();
}

void openhand()
{
	
	Gripper g;
	g.open_r_gripper();
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "right_arm_pick_place");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	putObjects(pub_co);
	
	/*
move_group_interface::MoveGroup group("right_arm");
	geometry_msgs::PoseStamped p;
	p.header.frame_id = "/base_footprint";
	p.pose.position.x = 0.59;
	p.pose.position.y = 0;
	p.pose.position.z = 0.825;
	p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, M_PI_4, M_PI_4);
group.setPoseTarget(p);
group.move();*/

//	Suturo_Manipulation_Planning_Scene_Interface pi(&nh);
	//moveit_msgs::PlanningScene ps;
	
	//move_group_interface::MoveGroup group("right_arm");	
	//group.setPlanningTime(45.0);
	/*pick(group);*/
	//Grasping grasper(&pi);
	//geometry_msgs::PoseStamped p;
	//p.header.frame_id = "/base_footprint";
	//p.pose.position.x = 0.59;
	//p.pose.position.y = 0;
	//p.pose.position.z = 0.625;
	//p.pose.orientation.x = 0;
	//p.pose.orientation.y = 0;
	//p.pose.orientation.z = 0;
	//p.pose.orientation.w = 1;
	
	//group.setPoseTarget(p);
	//group.move();
	//grasper.r_arm_pick("part", p);
	
	//move_group_interface::MoveGroup group("right_arm");
	//pick(group);
	//pi.attachObject("part", "r_wrist_roll_link");
	//pi.getPlanningScene(ps);
	//ROS_INFO_STREAM("ps " << ps);
	//ros::Publisher pub2 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
	//pub2.publish(ps);

	//ROS_INFO_STREAM("dsads  " << ps);
	
	ros::waitForShutdown();
	return 0;
}

/*
 * void spawnMesh(ros::Publisher pub_co)
{
  moveit_msgs::CollisionObject co2;
  co2.header.stamp = ros::Time::now();
  co2.header.frame_id = "/base_footprint";
    
  co2.id = "part2";
  co2.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co2);
  
  co2.operation = moveit_msgs::CollisionObject::ADD;
  co2.meshes.resize(1);
  co2.meshes[0].vertices.resize(4);
  co2.meshes[0].vertices[0].x = -0.9;
  co2.meshes[0].vertices[0].y = -0.9;
  co2.meshes[0].vertices[0].z = -0.9;
  
  co2.meshes[0].vertices[1].x = 1.5;
  co2.meshes[0].vertices[1].y = 1.5;
  co2.meshes[0].vertices[1].z = 1.5;
  
  co2.meshes[0].vertices[2].x = -0.5;
  co2.meshes[0].vertices[2].y = -0.5;
  co2.meshes[0].vertices[2].z = -0.5;
  
  co2.meshes[0].vertices[3].x = 1.0;
  co2.meshes[0].vertices[3].y = 1.0;
  co2.meshes[0].vertices[3].z = 1.0;
  
  co2.meshes[0].triangles.resize(4);
  co2.meshes[0].triangles[0].vertex_indices[0] = 0;
  co2.meshes[0].triangles[0].vertex_indices[1] = 1;
  co2.meshes[0].triangles[0].vertex_indices[2] = 2;
  
  co2.meshes[0].triangles[1].vertex_indices[0] = 1;
  co2.meshes[0].triangles[1].vertex_indices[1] = 2;
  co2.meshes[0].triangles[1].vertex_indices[2] = 3;
  
  co2.meshes[0].triangles[2].vertex_indices[0] = 0;
  co2.meshes[0].triangles[2].vertex_indices[1] = 1;
  co2.meshes[0].triangles[2].vertex_indices[2] = 3;
  
  co2.meshes[0].triangles[3].vertex_indices[0] = 0;
  co2.meshes[0].triangles[3].vertex_indices[1] = 2;
  co2.meshes[0].triangles[3].vertex_indices[2] = 3;
  
  co2.mesh_poses.resize(1);
  co2.mesh_poses[0].position.x =0.9;
  co2.mesh_poses[0].position.y =0;
  co2.mesh_poses[0].position.z =0.9;
  co2.mesh_poses[0].orientation.w=1.0;
  pub_co.publish(co2);	
}
*/
