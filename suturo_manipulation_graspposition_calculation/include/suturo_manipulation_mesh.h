#ifndef SUTURO_MANIPULATION_MESH
#define SUTURO_MANIPULATION_MESH

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <algorithm>

namespace suturo_manipulation
{

typedef std::pair< geometry_msgs::Point, geometry_msgs::Point > Plane_parameter;



struct Cluster
{
    std::vector<uint> triangles;
    std::vector<uint> vertices;
    geometry_msgs::Point normal;
};

class Plane
{
    Plane_parameter parameter_form;
    geometry_msgs::Point normal;
public:
    Plane(geometry_msgs::Point normal1, Plane_parameter parameter_form1);
    geometry_msgs::Point get_normal()
    {
        return normal;
    };
    Plane_parameter get_parameter_form()
    {
        return parameter_form;
    };
};

struct Vertex
{
    geometry_msgs::Point position;
    std::vector<uint> clusters;
    std::vector<uint> triangles;
    std::vector<uint> connected_vertices;
    bool operator==(const Vertex &v) const
    {
        return this->position.x == v.position.x
               && this->position.y == v.position.y
               && this->position.z == v.position.z;
    }
    bool operator!=(const Vertex &v) const
    {
        return !(*this == v);
    }
};

struct Triangle
{
    std::vector<uint> vertices;
    std::vector<uint> clusters;
    geometry_msgs::Point normal;
    std::string toString(std::vector<Cluster> clusterss, std::vector<Vertex> verticess)
    {
        std::ostringstream os;
        os << "vertices: (";
        for (int v = 0; v < vertices.size(); ++v)
        {
            os << vertices[v] << ",";
        }
        os << ")\n cluster: (";

        for (int v = 0; v < clusters.size(); ++v)
        {
            os << clusters[v] << ",";
        }
        os << ")";
        return "Triangle " + os.str();
    }
};

class Mesh
{

public:

    Mesh(shapes::Mesh *mesh);

    Mesh(moveit_msgs::CollisionObject co);

    ~Mesh();

    Cluster get_cluster(uint id);

    std::vector<Cluster> get_clusters();

    void add_cluster(Cluster c);

    Triangle get_triangle(uint id);

    std::vector<Triangle> get_triangles();

    void add_triangle(Triangle t);

    Vertex get_vertex(uint id);

    std::vector<Vertex> get_vertices();

    void add_vertex(Vertex v);

    Plane get_plane(uint cluster_id1, uint cluster_id2);

    int get_opposite_cluster_size();

    std::vector<geometry_msgs::Point> create_polygon(uint cluster_id);

    std::vector< std::pair<uint, uint> > get_opposite_cluster();

    void build_cluster(shapes::Mesh *mesh);

    void add_triangle_to_cluster(uint cluster_id, uint trianlge_id);

    bool is_vertex_boundary_point(uint vertex_id, uint cluster_id);

    bool is_vertex_in_cluster(uint vertex_id, uint cluster_id);

    void get_next_boundary_point(uint vertex_id, uint cluster_id, std::vector<uint> &next_vertex_ids);

    bool u_have_to_add_this_triangle(uint trianlge_id, std::vector<uint> connected_triangles, uint cluster_id);

    void set_connected_vertices();

    bool compare_vertex(uint vertex_id1, uint vertex_id2);

    void print();

    void print_vertex(uint vertex_id);

    template <class T>
    bool contains(std::vector<T> v, T elem);

    bool are_vertices_only_connected_by_one_triangle(uint vertex_id1, uint vertex_id2, uint cluster_id);

    static const double scalarproduct(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return (p1.x * p2.x) +
               (p1.y * p2.y) +
               (p1.z * p2.z);
    }

    static const double get_angle(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return acos (scalarproduct(p1, p2) /
                     (sqrt(scalarproduct(p1, p1)) * sqrt(scalarproduct(p2, p2))));
    }

protected:

    std::vector<Triangle> triangles;
    std::vector<Cluster> clusters;
    std::vector<Vertex> vertices;

    std::vector< std::pair<uint, uint> > opposite_cluster;

};
}

#endif
