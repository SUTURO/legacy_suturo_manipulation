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
	ros::WallDuration(2.0).sleep();
}

Suturo_Manipulation_Planning_Scene_Interface::~Suturo_Manipulation_Planning_Scene_Interface()
{

}

int Suturo_Manipulation_Planning_Scene_Interface::getPlanningScene(moveit_msgs::PlanningScene &ps)
{
	//msg to get Objectnames and Objectgeometry from planningscene
	moveit_msgs::GetPlanningScene msg;
	msg.request.components.components = 1023;
	
	//get planningscene
	ros::ServiceClient client = nh_->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	client.call(msg);
	if (client.call(msg))
	{
		ps = msg.response.scene;
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 0;
	}
	return 1;
}
/*
int Suturo_Manipulation_Planning_Scene_Interface::setPlanningScene(moveit_msgs::PlanningScene ps)
{
	
}*/

int Suturo_Manipulation_Planning_Scene_Interface::attachObject(std::string objectName, std::string linkName,
							std::vector<std::string> gripper_links)
{
	if (linkName.empty())
    {
      ROS_ERROR("No link specified to attach the object '%s' to", objectName.c_str());
      return false;
    }
    
  //subscribe to attach object topic
	moveit_msgs::AttachedCollisionObject attached_object;
	
	attached_object.link_name = linkName;
	getObject(objectName, attached_object.object);
	attached_object.object.operation = attached_object.object.ADD;
	
	attached_object.touch_links = gripper_links;
	
	attached_object_publisher_.publish(attached_object); 
	ros::WallDuration(2.0).sleep();
	return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::detachObject(std::string objectName)
{
    
  //subscribe to attach object topic
	moveit_msgs::AttachedCollisionObject attached_object;
	
	getObject(objectName, attached_object.object);
	attached_object.object.operation = attached_object.object.REMOVE;
	
	attached_object_publisher_.publish(attached_object); 
	ros::WallDuration(2.0).sleep();
	
	return 1;
}

/**
 * 
 */
int Suturo_Manipulation_Planning_Scene_Interface::getObject(std::string objectName, moveit_msgs::CollisionObject &co)
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
			if (tempCO.id == objectName){
				co = tempCO;
				break;
			}
		}
		//Object not found
		if (co.id != objectName){
			ROS_ERROR_STREAM("Object: " << objectName << " not found!!");
		}
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 0;
	}
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

int Suturo_Manipulation_Planning_Scene_Interface::getAttachedObject(std::string objectName, moveit_msgs::AttachedCollisionObject &co)
{
	std::vector<moveit_msgs::AttachedCollisionObject> attachedObjects = getAttachedObjects();
	for (int i = 0; i < attachedObjects.size(); i++){
		co = attachedObjects.at(i)
		if (co.id == objectName){
			return 1;
		}
	}
	ROS_ERROR_STREAM("Didn't found Object: " << objectName);
	return 0;
}



















