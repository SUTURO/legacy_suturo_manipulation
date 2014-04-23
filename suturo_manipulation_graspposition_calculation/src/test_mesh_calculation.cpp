#include <ros/ros.h>
#include <suturo_manipulation_grasp_calculator.h>
#include <suturo_manipulation_mesh_loader.h>
#include <clipper.h>

using namespace std;

bool test_build_cluster_corny(Grasp_Calculator *calc)
{
    Mesh_loader ml;
    shapes::Mesh *corny = ml.load_corny();
    std::vector<Grasp_Calculator::Cluster> clusters;
    calc->build_cluster(corny, clusters);

    bool t1 = clusters.size() == 6;

    ROS_INFO_STREAM("t1: " << t1);
    ROS_INFO_STREAM("corny triangle count = " << corny->triangle_count);
    ROS_INFO_STREAM("corny cluser count = " << clusters.size());

    shapes::Mesh *pancake = ml.load_pancake();
    std::vector<Grasp_Calculator::Cluster> clusters3;
    calc->build_cluster(pancake, clusters3);
    ROS_INFO_STREAM(pancake->triangle_count);
    ROS_INFO_STREAM(clusters3.size());

    std::vector< std::pair<Grasp_Calculator::Cluster, Grasp_Calculator::Cluster> > opposite_cluster;
    calc->search_for_opposte_cluster(clusters, opposite_cluster);
    ROS_INFO_STREAM("corny opposite cluster count = " << opposite_cluster.size());

    for (int i = 0; i < opposite_cluster.size(); i++)
    {
        Grasp_Calculator::Plane plane = calc->create_plane(opposite_cluster[i].first.normal, opposite_cluster[i].second.normal);
        ROS_INFO_STREAM("plane p1: " << plane.pp.first << "plane p2: " << plane.pp.second);
    }

    return false;
}

bool test_build_cluster_pringles(Grasp_Calculator *calc)
{
    Mesh_loader ml;
    shapes::Mesh *pringles = ml.load_pringles();
    std::vector<Grasp_Calculator::Cluster> clusters;
    calc->build_cluster(pringles, clusters);
    ROS_INFO_STREAM("pringles triangle count = " << pringles->triangle_count);
    ROS_INFO_STREAM("pringles cluser count = " << clusters.size());

    std::vector< std::pair<Grasp_Calculator::Cluster, Grasp_Calculator::Cluster> > opposite_cluster;
    calc->search_for_opposte_cluster(clusters, opposite_cluster);
    ROS_INFO_STREAM("pringles opposite cluster count = " << opposite_cluster.size());

    for (int i = 0; i < opposite_cluster.size(); i++)
    {
        Grasp_Calculator::Plane plane = calc->create_plane(opposite_cluster[i].first.normal, opposite_cluster[i].second.normal);
        ROS_INFO_STREAM("plane p1: " << plane.pp.first << "plane p2: " << plane.pp.second);
    }

}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "test_mesh_calculation");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    Suturo_Manipulation_Planning_Scene_Interface pi(&nh);
    Grasp_Calculator calc(&pi);
    test_build_cluster_corny(&calc);
    test_build_cluster_pringles(&calc);
}
