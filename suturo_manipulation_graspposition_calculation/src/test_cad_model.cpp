#include <ros/ros.h>
#include <suturo_manipulation_grasp_calculator.h>
#include <suturo_manipulation_mesh_loader.h>
#include <clipper.h>
#include <suturo_manipulation_mesh.h>

using namespace std;
using namespace ClipperLib;
using namespace suturo_manipulation;

int main(int argc, char **argv)
{
    ros::init (argc, argv, "test_mesh_calculation");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;
    if (argc == 0)
    {
        ROS_INFO_STREAM("add name of file.");
        return 0;
    }

    string path = argv[1];
    path = "package://suturo_manipulation_graspposition_calculation/test_cad_models/models/" + path;

    Mesh_loader ml;
    Suturo_Manipulation_Planning_Scene_Interface pi(&nh);
    Grasp_Calculator calc(&pi);

    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<geometry_msgs::PoseStamped> pre_poses;

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "/base_footprint";
    co.id = "pancake";

    co.meshes.resize(1);
    co.meshes[0] = ml.load_mesh_msg(path);
    co.mesh_poses.resize(1);
    co.mesh_poses[0].position.x = 1;
    co.mesh_poses[0].position.y = 0;
    co.mesh_poses[0].position.z = 0;
    co.mesh_poses[0].orientation.w = 1;
    pi.addObject(co);
    geometry_msgs::PoseStamped ps;
    ps.header = co.header;
    ps.pose = co.mesh_poses[0];
    calc.calcMeshGraspPosition(co, poses, pre_poses, Gripper::R_GRIPPER_PALM_LENGTH);
}
