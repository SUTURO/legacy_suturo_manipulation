#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>
#include <string>

#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "suturo_manipulation_planning_scene_interface.h"

Suturo_Manipulation_Planning_Scene_Interface::Suturo_Manipulation_Planning_Scene_Interface(ros::NodeHandle* nodehandle)
{
	nh_ = nodehandle;
	attached_object_publisher_ = nh_->advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	collision_object_publisher_ = nh_->advertise<moveit_msgs::CollisionObject>("collision_object", 10);	
	
	//wait because ros
	ros::WallDuration(1.0).sleep();
}

Suturo_Manipulation_Planning_Scene_Interface::~Suturo_Manipulation_Planning_Scene_Interface()
{

}

int Suturo_Manipulation_Planning_Scene_Interface::getPlanningScene(moveit_msgs::PlanningScene &ps)
{
	//create msg to get Objectnames and Objectgeometry from planningscene
	moveit_msgs::GetPlanningScene msg;
	msg.request.components.components = 1023;
	
	//get planningscene
	ROS_INFO_STREAM("call service for planningscene");
	ros::ServiceClient client = nh_->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	client.call(msg);
	if (client.call(msg))
	{
		ps = msg.response.scene;
	}
	else
	{
		ROS_ERROR("Suturo_Manipulation_Planning_Scene_Interface::getPlanningScene| Failed to call service.");
		return 0;
	}
	return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::attachObject(std::string object_name, std::string link_name,
							std::vector<std::string> gripper_links)
{
	//check if the link name is valid
	if (link_name.empty())
	{
		ROS_ERROR("Suturo_Manipulation_Planning_Scene_Interface::attachObject| No link specified to attach the object '%s' to", 
				object_name.c_str());
		return false;
	}
  
  //get object from the planningscene
	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = link_name;
	getObject(object_name, attached_object.object);
	attached_object.object.operation = attached_object.object.ADD;
	
	//specify the links that are allowed to touch the object
	attached_object.touch_links = gripper_links;
	
	//tell the planningscene that the object is attached
	attached_object_publisher_.publish(attached_object);
	
	//wait because ros
	ros::WallDuration(1.0).sleep();
	ROS_INFO_STREAM("attached " << object_name << " to " << link_name);
	return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getObject(std::string object_name, moveit_msgs::CollisionObject &co)
{

	//msg to get Objectnames and Objectgeometry from planningscene
	moveit_msgs::GetPlanningScene msg;
	msg.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY +
				moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
	
	//get planningscene
	ros::ServiceClient client = nh_->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	client.call(msg);
	if (client.call(msg))
	{
		//search for objectname
		for(int i = 0; i < msg.response.scene.world.collision_objects.size(); i++){
			moveit_msgs::CollisionObject tempCO = msg.response.scene.world.collision_objects[i];
			if (tempCO.id == object_name){
				co = tempCO;
				break;
			}
		}
		//Object not found
		if (co.id != object_name){
			ROS_ERROR_STREAM("Suturo_Manipulation_Planning_Scene_Interface::getObject| Object: " << object_name << " not found!!");
			return 0;
		}
	}
	else
	{
		ROS_ERROR("Suturo_Manipulation_Planning_Scene_Interface::getObject| Failed to call service move_group::GET_PLANNING_SCENE_SERVICE_NAME");
		return 0;
	}
	ROS_INFO_STREAM("object " << object_name << " found.");
	return 1;
}

std::vector<moveit_msgs::AttachedCollisionObject> Suturo_Manipulation_Planning_Scene_Interface::getAttachedObjects()
{
	moveit_msgs::PlanningScene ps;
	if (!getPlanningScene(ps)){
		ROS_ERROR_STREAM("Failed to get planningscene");
	}
	return ps.robot_state.attached_collision_objects;
}

int Suturo_Manipulation_Planning_Scene_Interface::getAttachedObject(std::string object_name, moveit_msgs::AttachedCollisionObject &co)
{
	std::vector<moveit_msgs::AttachedCollisionObject> attachedObjects = getAttachedObjects();
	//search for object_name in the list of attached objects
	for (int i = 0; i < attachedObjects.size(); i++){
		co = attachedObjects.at(i);
		if (co.object.id == object_name){
			return 1;
		}
	}
	return 0;
}


int Suturo_Manipulation_Planning_Scene_Interface::addObject(moveit_msgs::CollisionObject co)
{
	co.operation = moveit_msgs::CollisionObject::REMOVE;
  collision_object_publisher_.publish(co);
  
	co.operation = moveit_msgs::CollisionObject::ADD;
  collision_object_publisher_.publish(co);
  ROS_INFO_STREAM("Added object " << co.id << " to planningscene.");
	return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::removeObject(moveit_msgs::CollisionObject co)
{
	co.operation = moveit_msgs::CollisionObject::REMOVE;
  collision_object_publisher_.publish(co);
  ROS_INFO_STREAM("Removed " << co.id << " form planningscene.");
  return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::detachObject(std::string object_name)
{
  
	moveit_msgs::AttachedCollisionObject attached_object;
	moveit_msgs::AttachedCollisionObject detached_object;
	
	//get the attached object
	if (!getAttachedObject(object_name, attached_object)){
		ROS_INFO_STREAM(object_name << " wasn't attached");
		return 1;
	}
	detached_object.object.operation = attached_object.object.REMOVE;
	detached_object.object.id = attached_object.object.id;
	detached_object.link_name = attached_object.link_name;
	//detach it
	attached_object_publisher_.publish(detached_object); 
	
	//wait because ros
	ros::WallDuration(1.0).sleep();
	ROS_INFO_STREAM(object_name << " detached.");
	return 1;
}

bool Suturo_Manipulation_Planning_Scene_Interface::isObjectAttached(std::string object_name)
{
	moveit_msgs::AttachedCollisionObject co;
	return getAttachedObject(object_name, co);
}

















