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


/**
 * This Programm publishes a tf frame into every collisionobject and attached object.
 */

void publishTfFrame(std::string frame_id, geometry_msgs::PoseStamped pose, 
		tf::Transform transform, tf::TransformBroadcaster br)
{

  ROS_DEBUG_STREAM("Publish TF frame " << frame_id);
  
  transform.setOrigin( tf::Vector3(pose.pose.position.x,	pose.pose.position.y, pose.pose.position.z) );
				
  transform.setRotation( tf::Quaternion(pose.pose.orientation.x,
					pose.pose.orientation.y,
					pose.pose.orientation.z,
					pose.pose.orientation.w) );

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), pose.header.frame_id, frame_id));
}

geometry_msgs::PoseStamped getCamPose(ros::NodeHandle n){
	geometry_msgs::PoseStamped cam_pose;
	
	//set orientation
	cam_pose.pose.orientation.w = 1;
	
	if (!n.getParam("/suturo_manipulation_tf_publisher/cam_frame", cam_pose.header.frame_id)){
		ROS_ERROR_STREAM("Failed to Frame for Cam.");
	}
	if (!n.getParam("/suturo_manipulation_tf_publisher/cam_x", cam_pose.pose.position.x)){
		ROS_ERROR_STREAM("Failed to get x coordinate for Cam.");
	}
	if (!n.getParam("/suturo_manipulation_tf_publisher/cam_y", cam_pose.pose.position.y)){
		ROS_ERROR_STREAM("Failed to get y coordinate for Cam.");
	}
	if (!n.getParam("/suturo_manipulation_tf_publisher/cam_z", cam_pose.pose.position.z)){
		ROS_ERROR_STREAM("Failed to get z coordinate for Cam.");
	}
	ROS_INFO_STREAM(cam_pose);
	return cam_pose;
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
	
	boost::this_thread::sleep(boost::posix_time::seconds(3));
	
	geometry_msgs::PoseStamped cam_pose = getCamPose(n);
	
	geometry_msgs::PoseStamped temp_pose;
	ROS_INFO_STREAM("tf publisher started..........");
	
  while(pi.getObjects(cos) && pi.getAttachedObjects(acos))
  {
		//publish tf frame in every collisionobject
		for (std::vector<moveit_msgs::CollisionObject>::iterator co = cos.begin(); co != cos.end(); ++co){
			temp_pose.pose = co->primitive_poses[0];
			temp_pose.header = co->header;
			publishTfFrame(co->id, temp_pose, transform, br);
		}
		
		//publish tf frame in every attachedobject
		for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator aco = acos.begin(); aco != acos.end(); ++aco){
			ROS_DEBUG_STREAM(*aco);
			temp_pose.pose = aco->object.primitive_poses[0];
			temp_pose.header = aco->object.header;
			publishTfFrame(aco->object.id, temp_pose, transform, br);
		}
		
		//publish cam_frame
		publishTfFrame("webcam", cam_pose, transform, br);
		
		boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}


















