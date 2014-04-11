#include "suturo_manipulation_mesh_loader.h"

const std::string Mesh_loader::CORNY_PATH = "package://suturo_perception_cad_recognition/test_data/corny.ply";
const std::string Mesh_loader::PRINGLES_PATH = "package://suturo_perception_cad_recognition/test_data/pringles.ply";


shape_msgs::Mesh Mesh_loader::load_mesh_msg(std::string resource)
{
    shape_msgs::Mesh mesh_msg;
    shapes::ShapeMsg mesh;
    shapes::constructMsgFromShape( shapes::createMeshFromResource(resource), mesh);
    mesh_msg = boost::get<shape_msgs::Mesh>(mesh);
    return mesh_msg;
}

shapes::Mesh* Mesh_loader::load_mesh(std::string resource)
{
	return shapes::createMeshFromResource(resource);
}

shape_msgs::Mesh Mesh_loader::load_corny_msg()
{
    return load_mesh_msg(CORNY_PATH);
}

shapes::Mesh* Mesh_loader::load_corny()
{
	return shapes::createMeshFromResource(CORNY_PATH);
}

shape_msgs::Mesh Mesh_loader::load_pringles_msg()
{
    return load_mesh_msg(PRINGLES_PATH);
}

shapes::Mesh* Mesh_loader::load_pringles()
{
	return shapes::createMeshFromResource(PRINGLES_PATH);
}



















