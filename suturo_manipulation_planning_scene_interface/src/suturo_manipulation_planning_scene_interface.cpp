#include "suturo_manipulation_planning_scene_interface.h"

Suturo_Manipulation_Planning_Scene_Interface::Suturo_Manipulation_Planning_Scene_Interface(ros::NodeHandle* nodehandle)
{
	nh_ = nodehandle;
	attached_object_publisher_ = nh_->advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	collision_object_publisher_ = nh_->advertise<moveit_msgs::CollisionObject>("collision_object", 10);	
	vis_pub_ = nh_->advertise<visualization_msgs::Marker>( "/suturo/visualization_marker", 10 );
	
	//wait because ros
	ros::WallDuration(0.5).sleep();
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
	ros::ServiceClient client = nh_->serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	client.call(msg);
	if (client.call(msg))
	{
		ps = msg.response.scene;
	}
	else
	{
		ROS_ERROR("Failed to call service to get planningscene.");
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
		ROS_ERROR("No link specified to attach the object '%s' to", 
				object_name.c_str());
		return false;
	}
  
  //check if another object is attached to this link
  std::vector<moveit_msgs::AttachedCollisionObject> acos;
  if (!getAttachedObjects(acos)) return 0;
	
	for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = acos.begin(); it != acos.end(); ++it){
		if (it->object.id == object_name){
			ROS_WARN_STREAM(object_name << " already attached to " << it->link_name << ".");
			if (it->link_name == link_name){
				ROS_WARN_STREAM(object_name << " already attached to this link.");
				return 1;
			} else {
				ROS_INFO_STREAM("Detaching object from old link.");
				detachObject(object_name);
			}
		}
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
	ROS_DEBUG_STREAM("Attached " << object_name << " to " << link_name << ".");
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
			ROS_WARN_STREAM(" Object: " << object_name << " not found!!");
			return 0;
		}
	}
	else
	{
		ROS_ERROR("Suturo_Manipulation_Planning_Scene_Interface::getObject| Failed to call service move_group::GET_PLANNING_SCENE_SERVICE_NAME");
		return 0;
	}
	ROS_DEBUG_STREAM("Object " << object_name << " found.");
	return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getAttachedObjects(std::vector<moveit_msgs::AttachedCollisionObject> &acos)
{
	moveit_msgs::PlanningScene ps;
	if (!getPlanningScene(ps)){
		ROS_ERROR_STREAM("Failed to get planningscene");
		return 0;
	}
	acos = ps.robot_state.attached_collision_objects;
	return 1;
}

int Suturo_Manipulation_Planning_Scene_Interface::getObjects(std::vector<moveit_msgs::CollisionObject> &cos)
{
	moveit_msgs::PlanningScene ps;
	if (!getPlanningScene(ps)){
		ROS_ERROR_STREAM("Failed to get planningscene");
		return 0;
	}
	cos = ps.world.collision_objects;
	return 1;
}

std::vector<moveit_msgs::CollisionObject> Suturo_Manipulation_Planning_Scene_Interface::getObjects()
{
	moveit_msgs::PlanningScene ps;
	if (!getPlanningScene(ps)){
		ROS_ERROR_STREAM("Failed to get planningscene");
	}
	return ps.world.collision_objects;
}

int Suturo_Manipulation_Planning_Scene_Interface::getAttachedObject(std::string object_name, moveit_msgs::AttachedCollisionObject &co)
{
	std::vector<moveit_msgs::AttachedCollisionObject> attachedObjects;
	if (!getAttachedObjects(attachedObjects)) return 0;
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
		ROS_INFO_STREAM(object_name << " wasn't attached.");
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

int Suturo_Manipulation_Planning_Scene_Interface::isAnObjectAttachedToArm(std::string link_name)
{
	
	std::vector<moveit_msgs::AttachedCollisionObject> acos;
	if(!getAttachedObjects(acos)) return 0;
	
	for (std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = acos.begin(); it != acos.end(); ++it){
		if (it->link_name == link_name){
			//if the link names are the same, the object is attached to it
			return 1;
		} 
	}
	
	return 0;
}

void Suturo_Manipulation_Planning_Scene_Interface::publishMarker(geometry_msgs::PoseStamped pose)
 {
// Publish the Goalmarker	
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = pose.header.frame_id;
  goal_marker.header.stamp = ros::Time();
  goal_marker.ns = "suturo_manipulation";
  goal_marker.id = 0;
// <<<<<<< HEAD
//   goal_marker.type = visualization_msgs::Marker::SPHERE;
//   goal_marker.action = visualization_msgs::Marker::ADD;
//   goal_marker.pose.position.x = pose.pose.position.x;
//   goal_marker.pose.position.y = pose.pose.position.y;
//   goal_marker.pose.position.z = pose.pose.position.z;
//   goal_marker.pose.orientation.x = 0.0;
//   goal_marker.pose.orientation.y = 0.0;
//   goal_marker.pose.orientation.z = 0.0;
//   goal_marker.pose.orientation.w = 1.0;
//   goal_marker.scale.x = 0.1;
//   goal_marker.scale.y = 0.1;
//   goal_marker.scale.z = 0.1;
//   goal_marker.color.a = 1.0;
//   goal_marker.color.r = 0.0;
//   goal_marker.color.g = 1.0;
//   goal_marker.color.b = 0.0;
// =======
  goal_marker.type = visualization_msgs::Marker::ARROW;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose = pose.pose;
  
  goal_marker.scale.x = 0.1;
  goal_marker.scale.y = 0.1;
  goal_marker.scale.z = 0.1;
  
  goal_marker.color.a = 1.0;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  
  goal_marker.scale.z = 0.04;
  goal_marker.scale.y = 0.04;
  goal_marker.scale.x = 0.09;

  vis_pub_.publish( goal_marker );   
}  
















