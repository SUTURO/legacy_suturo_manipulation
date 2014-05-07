#include <ros/ros.h>
#include <suturo_manipulation_grasp_calculator.h>
#include <suturo_manipulation_mesh_loader.h>
#include <clipper.h>
#include <suturo_manipulation_mesh.h>

using namespace std;
using namespace ClipperLib;
using namespace suturo_manipulation;

std::ostream &operator <<(std::ostream &s, const Grasp_Calculator::DoublePoint2D &p)
{
    s << "(" << p.x << "," << p.y << ")";
    return s;
}

template <class T>
void print_vector(std::vector<T> v)
{
    for (int i = 0; i < v.size(); ++i)
    {
        ROS_INFO_STREAM(v[i]);
    }
}

template <class T>
bool comp_vector(std::vector<T> v, std::vector<T> v2)
{
    if (v.size() != v2.size()) return false;
    for (int i = 0; i < v.size(); ++i)
    {
        if (v[i] != v2[i]) return false;

    }
    return true;
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

    meshi.print();

    //TEST 1
    uint a = 0;
    for (int cluster_id = 0; cluster_id < clusters.size(); ++cluster_id)
    {
        a += clusters[cluster_id].triangles.size();
    }
    // ROS_WARN_STREAM("cluster check 1: " << (a == meshi.get_triangles().size()) );

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
    }
    // ROS_WARN_STREAM("cluster " << c_id << " vertex count check: " << (v.size() == clusters[c_id].vertices.size()));



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

    //TEST4
    for (uint t_id = 0; t_id < triangles.size(); ++t_id)
    {
        if (!meshi.contains(vertices[(triangles[t_id].vertices[0])].triangles, t_id))
            ROS_INFO_STREAM("muh: " << t_id);
    }

    for (uint t_id = 0; t_id < vertices.size(); ++t_id)
    {
        if (!meshi.contains(triangles[(vertices[t_id].triangles[0])].vertices, t_id))
            ROS_INFO_STREAM("muh2: " << t_id);
    }

    //TEST 5
    for (uint v_id = 0; v_id < vertices.size(); ++v_id)
    {
        // ROS_ERROR_STREAM("Vertex " << v_id << " size " << vertices[v_id].connected_vertices.size());
        for (uint c_v_id = 0; c_v_id < vertices[v_id].connected_vertices.size(); ++c_v_id)
        {
            uint c_v_id_real = vertices[v_id].connected_vertices[c_v_id];
            if (!meshi.contains(vertices[c_v_id_real].connected_vertices, v_id) )
                // {
                ROS_ERROR_STREAM("connected_vertices kaputt!! " << c_v_id_real);
            // ROS_INFO_STREAM("id: " << c_v_id_real);
            // print_vector(vertices[c_v_id_real].connected_vertices);
            // ROS_INFO_STREAM("----");
            // }

        }
        // ROS_INFO_STREAM("=========");
    }



    std::vector< std::pair<uint, uint> > opposite_cluster = meshi.get_opposite_cluster();

    ROS_INFO_STREAM(name << " opposite cluster count = " << opposite_cluster.size());

    for (int i = 0; i < opposite_cluster.size(); i++)
    {
        ROS_INFO_STREAM("opposite cluster  " << opposite_cluster[i].first << "   " << opposite_cluster[i].second);
        suturo_manipulation::Plane plane = meshi.get_plane(opposite_cluster[i].first, opposite_cluster[i].second);
        ROS_INFO_STREAM(name << " plane : " << plane.get_normal() );
        std::vector<geometry_msgs::Point> p1 = meshi.create_polygon(opposite_cluster[i].first);
        std::vector<geometry_msgs::Point> p2 = meshi.create_polygon(opposite_cluster[i].second);

        Grasp_Calculator::DPolygon2D dpolygon1 = calc->project_polygon_to_plane(plane, p1);
        // for (std::vector<Grasp_Calculator::DoublePoint2D>::iterator p2d = dpolygon1.begin(); p2d != dpolygon1.end(); ++p2d)
        // {
        //     ROS_INFO_STREAM("    a = " << p2d->x << " b= " << p2d->y << std::endl);
        // }
        Grasp_Calculator::DPolygon2D dpolygon2 = calc->project_polygon_to_plane(plane, p2);
        // ROS_ERROR_STREAM("???");
        ClipperLib::Paths polygon1;
        ClipperLib::Paths polygon2;
        calc->double_polygon_to_path(dpolygon1, polygon1);
        if (polygon1[0].size() != dpolygon1.size()) ROS_WARN_STREAM("dpoly und poly size doen't match!!!");
        calc->double_polygon_to_path(dpolygon2, polygon2);
        if (polygon2[0].size() != dpolygon2.size()) ROS_WARN_STREAM("dpoly und poly size doen't match!!!");

        // Grasp_Calculator::DPolygon2D test1;
        // Grasp_Calculator::DPolygon2D test2;
        // calc->path_to_double_polygon(test1, polygon1[0]);
        // calc->path_to_double_polygon(test2, polygon2[0]);
        // if (!comp_vector(test1, dpolygon1))
        // {
        //     ROS_WARN_STREAM("path to polygon fail1");
        //     print_vector(test1);
        //     ROS_INFO_STREAM("");
        //     print_vector(dpolygon1);
        // }
        // if (!comp_vector(test2, dpolygon2)) ROS_WARN_STREAM("path to polygon fail2");


        ClipperLib::Clipper clpr;
        clpr.AddPaths(polygon1, ClipperLib::ptSubject, true);
        clpr.AddPaths(polygon2, ClipperLib::ptClip, true);
        ClipperLib::Paths solution;
        clpr.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);


        Path centroids = calc->calc_poly_centroid(solution);
        Grasp_Calculator::DPolygon2D centroids3d;
        calc->path_to_double_polygon(centroids3d, centroids);
        ROS_INFO_STREAM("centroid count: " << centroids3d.size());
        // for (std::vector<Grasp_Calculator::DoublePoint2D>::iterator s = centroids3d.begin(); s != centroids3d.end(); ++s)
        // {
        //     ROS_INFO_STREAM("first centroid");
        //     ROS_INFO_STREAM("x = " << s->x << " y = " << s->y);
        //     // geometry_msgs::Point p = calc->d2d_point_to_d3d_point(plane, *s);
        //     // ROS_INFO_STREAM("x = " << p.x << " y = " << p.y << " z= " << p.z << std::endl);
        // }
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
    // test(corny, "corny", &calc);
    shapes::Mesh *pringles = ml.load_pringles();
    // test(pringles, "pringles");
    shapes::Mesh *pancake = ml.load_pancake();
    test(pancake, "pancake", &calc);
}
