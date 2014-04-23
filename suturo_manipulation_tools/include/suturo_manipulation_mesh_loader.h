#ifndef SUTURO_MANIPULATION_MESH_LOADER
#define SUTURO_MANIPULATION_MESH_LOADER

#include <ros/ros.h>

// #include <clipper.h>

#include <suturo_manipulation_planning_scene_interface.h>

#include <moveit/move_group_interface/move_group.h>

#include <shape_tools/solid_primitive_dims.h>

// #include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
// #include <shapes.h>

class Mesh_loader{

protected:
	const static std::string CORNY_PATH;
	const static std::string PRINGLES_PATH;
	const static std::string PANCAKE_MODEL_PATH;

public:
	shape_msgs::Mesh load_mesh_msg(std::string resource);
	shapes::Mesh* load_mesh(std::string resource);

	shape_msgs::Mesh load_corny_msg();	
	shapes::Mesh* load_corny();	

	shape_msgs::Mesh load_pringles_msg();	
	shapes::Mesh* load_pringles();

	shape_msgs::Mesh load_pancake_msg();	
	shapes::Mesh* load_pancake();	

};

#endif
















