#include "ros/ros.h"
#include <suturo_manipulation_planning_scene_interface.h>
#include <tf/transform_broadcaster.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_planning_scene_interface.h>

void publishTfFrame(moveit_msgs::CollisionObject co, tf::Transform transform, tf::TransformBroadcaster br)
{

  ROS_DEBUG_STREAM("publish object frame " << co.id);
  
  transform.setOrigin( tf::Vector3(co.primitive_poses[0].position.x, 
				co.primitive_poses[0].position.y, co.primitive_poses[0].position.z) );
				
  transform.setRotation( tf::Quaternion(co.primitive_poses[0].orientation.x,
					co.primitive_poses[0].orientation.y,
					co.primitive_poses[0].orientation.z,
					co.primitive_poses[0].orientation.w) );
					
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", co.id));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_objects_tf_frames");
  ros::NodeHandle n;
	
	tf::Transform transform;
	tf::TransformBroadcaster br;

	Suturo_Manipulation_Planning_Scene_Interface pi(&n);
	std::vector<moveit_msgs::CollisionObject> cos;
	std::vector<moveit_msgs::AttachedCollisionObject> acos;

  while(true)
  {
    cos = pi.getObjects();
		for (std::vector<moveit_msgs::CollisionObject>::iterator co = cos.begin(); co != cos.end(); ++co){
			publishTfFrame(*co, transform, br);
		}
		
    acos = pi.getAttachedObjects();
		for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator aco = acos.begin(); aco != acos.end(); ++aco){
			publishTfFrame(aco->object, transform, br);
		}
		
		boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}


















