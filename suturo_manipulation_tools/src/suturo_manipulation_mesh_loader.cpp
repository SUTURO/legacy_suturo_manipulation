#include "suturo_manipulation_mesh_loader.h"

shape_msgs::Mesh Mesh_loader::load_mesh(std::string resource){
	shape_msgs::Mesh mesh_msg;
	shapes::ShapeMsg mesh;
	shapes::constructMsgFromShape( shapes::createMeshFromResource(resource), mesh);
	mesh_msg = boost::get<shape_msgs::Mesh>(mesh);
	return mesh_msg;
}

shape_msgs::Mesh Mesh_loader::load_corny(){
	return load_mesh("package://suturo_perception_cad_recognition/test_data/corny.ply");
}

shape_msgs::Mesh Mesh_loader::load_pringles(){
	return load_mesh("package://suturo_perception_cad_recognition/test_data/pringles.ply");
}


















