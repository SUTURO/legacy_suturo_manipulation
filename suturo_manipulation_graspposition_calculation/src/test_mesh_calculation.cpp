#include <ros/ros.h>
#include <suturo_manipulation_grasp_calculator.h>
#include <suturo_manipulation_mesh_loader.h>
#include <clipper.h>
#include <suturo_manipulation_mesh.h>

using namespace std;

template <class T>
void print_vector(std::vector<T> v)
{
    for (int i = 0; i < v.size(); ++i)
    {
        ROS_INFO_STREAM(v[i]);
    }
}

bool test(shapes::Mesh *mesh, std::string name, Grasp_Calculator *calc)
{
    Mesh_loader ml;
    shapes::Mesh *pringles = ml.load_pringles();
    suturo_manipulation::Mesh meshi(mesh);

    std::vector<suturo_manipulation::Cluster> clusters = meshi.get_clusters();
    std::vector<suturo_manipulation::Triangle> triangles = meshi.get_triangles();
    std::vector<suturo_manipulation::Vertex> vertices = meshi.get_vertices();

    ROS_INFO_STREAM(name << " triangle count = " << meshi.get_triangles().size());
    ROS_INFO_STREAM(name << " cluster count = " << clusters.size());

    // meshi.print();

    //TEST 1
    uint a = 0;
    for (int cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
    {
        a += clusters[cluster_id].triangles.size();
    }
    ROS_WARN_STREAM("cluster check 1: " << (a == meshi.get_triangles().size()) );

    //TEST 1.1
    for (std::vector<suturo_manipulation::Triangle>::iterator t = triangles.begin(); t != triangles.end(); ++t)
    {
        if (t->clusters.size() != 1)
            ROS_ERROR_STREAM("Triangle is in more or less then 1 Cluster!");
    }

    a = 0;

    //TEST 2
    for (int c_id = 0; c_id < clusters.size(); ++c_id)
    {
        std::vector<uint> v;
        for (std::vector<uint>::iterator triangle_id = clusters[c_id].triangles.begin(); triangle_id != clusters[c_id].triangles.end(); ++triangle_id)
        {
            for (std::vector<uint>::iterator vertex_id = triangles[*triangle_id].vertices.begin(); vertex_id != triangles[*triangle_id].vertices.end(); ++vertex_id)
            {

                if (!meshi.contains(v, *vertex_id))
                {
                    v.push_back(*vertex_id);
                }
            }
        }

        ROS_WARN_STREAM("cluster " << c_id << " vertex count check: " << (v.size() == clusters[c_id].vertices.size()));

    }

    //TEST 3
    for (std::vector<suturo_manipulation::Cluster>::iterator c = clusters.begin(); c != clusters.end(); ++c)
    {
        uint c_id = c - clusters.begin();
        // ROS_INFO_STREAM("cluster vertices: " << c_id);
        // print_vector(c->vertices);
        for (std::vector<uint>::iterator v_id = c->vertices.begin(); v_id != c->vertices.end(); ++v_id)
        {
            // ROS_INFO_STREAM("vertex clusters: " << *v_id);
            // print_vector(vertices[*v_id].clusters);
            if (!meshi.contains(vertices[*v_id].clusters, c_id))
                ROS_ERROR_STREAM("Vertex cluster entry inconsistent!!!");
        }
    }




    std::vector< std::pair<uint, uint> > opposite_cluster = meshi.get_opposite_cluster();

    // ROS_INFO_STREAM(name << " opposite cluster count = " << opposite_cluster.size());

    // for (int i = 0; i < clusters.size(); ++i)
    // {
    //     ROS_INFO_STREAM("Cluster Polygon: " << i);
    //     print_vector(meshi.create_polygon(i));
    // }

    for (int i = 0; i < opposite_cluster.size(); i++)
    {
        suturo_manipulation::Plane plane = meshi.get_plane(opposite_cluster[i].first, opposite_cluster[i].second);
        ROS_INFO_STREAM(name << " plane normal: " << plane.normal);
        std::vector<geometry_msgs::Point> p1 = meshi.create_polygon(opposite_cluster[i].first);
        std::vector<geometry_msgs::Point> p2 = meshi.create_polygon(opposite_cluster[i].second);

        Grasp_Calculator::DPolygon2D dpolygon1 = calc->project_polygon_to_plane(plane, p1);
        Grasp_Calculator::DPolygon2D dpolygon2 = calc->project_polygon_to_plane(plane, p2);
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
    Mesh_loader ml;
    shapes::Mesh *corny = ml.load_corny();
    test(corny, "corny", &calc);
    shapes::Mesh *pringles = ml.load_pringles();
    // test(pringles, "pringles");
    shapes::Mesh *pancake = ml.load_pancake();
    // test(pancake, "pancake");
}
