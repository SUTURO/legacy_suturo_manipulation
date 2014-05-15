#ifndef SUTURO_MANIPULATION_MESH
#define SUTURO_MANIPULATION_MESH

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <algorithm>

namespace suturo_manipulation
{

struct Cluster
{
    std::vector<uint> triangles;
    std::vector<uint> vertices;
    std::vector<geometry_msgs::Point> bounding_polygon;
    geometry_msgs::Point normal;
};

class Plane
{
    geometry_msgs::Point support_vector_;
    geometry_msgs::Point first_parameter_;
    geometry_msgs::Point second_parameter_;
    geometry_msgs::Point normal_;

    //4th parameter of koordinate form
    double d;

    //for 2d point calculation
    geometry_msgs::Point u_a;
    geometry_msgs::Point u_b;
public:
    Plane(geometry_msgs::Point normal,
          geometry_msgs::Point support_vector,
          geometry_msgs::Point first_parameter,
          geometry_msgs::Point second_parameter);

    geometry_msgs::Point get_normal()
    {
        return normal_;
    };

    /**
    * Orthonormalizes the vectors of the parameterform. Calls pre_compute afterwards.
    * After this, the value of the vactor is the same in 2D and 3D space.
    *
    * @return false, if it fails
    */
    bool orthonormalize();

    /**
    * Precomputes parameters for fast 3D to 2D point calculation.
    *
    * @return false, if divison by zero.
    */
    bool pre_compute();

    void get_parameter_form(geometry_msgs::Point &support_vector,
                            geometry_msgs::Point &first_parameter,
                            geometry_msgs::Point &second_parameter)
    {
        support_vector = support_vector_;
        first_parameter = first_parameter_;
        second_parameter = second_parameter_;
    };

    /**
    * Calculates the point of intersection between a line and the plane.
    *
    * @return false, if divison by zero.
    */
    bool get_point_of_intersection(geometry_msgs::Point &result, geometry_msgs::Point s, geometry_msgs::Point r, double &lamda);


    /**
    * Transforms a 3D point, to a Plane point.
    *
    * @return false, if it fails.
    */
    bool d3d_point_to_d2d_point(double &a, double &b, geometry_msgs::Point x);

    void get_coordinate_form(double &a, double &b, double &c, double &d);

    /**
    * Calculates the distance to the triangle, that is given by the parameterform.
    *
    * @return false, if it fails or r would be negativ.
    */
    bool dist_ray_triangle(geometry_msgs::Point s, geometry_msgs::Point r, double &dist);

    /**
    * Calculates the distance to the triangle, that is given by the parameterform.
    *
    * @return false, if it fails.
    */
    bool dist_line_triangle(geometry_msgs::Point s, geometry_msgs::Point r, double &dist);

    void project_point_to_plane(geometry_msgs::Point p, double &x, double &y);


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
    Plane *plane_;

    void calc_plane(std::vector<Vertex> vertices1);

    std::string toString(std::vector<Cluster> clusterss, std::vector<Vertex> verticess)
    {
        std::ostringstream os;
        os << "vertices: (";
        for (int v = 0; v < vertices.size(); ++v)
        {
            os << vertices[v] << ",";
        }
        os << ")\n cluster: (";

        // for (int v = 0; v < clusters.size(); ++v)
        // {
        //     os << clusters[v] << ",";
        // }
        os << ")";
        return "Triangle " + os.str();
    };


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

    /**
    * Calculates a plane, that lies between to clusters.
    *
    * @return a plane
    */
    Plane get_plane(uint cluster_id1, uint cluster_id2);

    int get_opposite_cluster_size();

    /**
    * Calculates a polygon that surrounds the cluster.
    *
    * @return a polygon
    */
    void create_polygon(uint cluster_id);
    
    std::vector<geometry_msgs::Point> get_polygon(uint cluster_id);



    std::vector< std::pair<uint, uint> > get_opposite_cluster();

    /**
     * Computes Clusters.
     *
     */
    void build_cluster(shapes::Mesh *mesh);

    void add_triangle_to_cluster(uint cluster_id, uint trianlge_id);

    bool is_vertex_boundary_point(uint vertex_id, uint cluster_id);

    bool is_vertex_in_cluster(uint vertex_id, uint cluster_id);

    void get_next_boundary_point(uint vertex_id, uint cluster_id, std::vector<uint> &next_vertex_ids);

    bool u_have_to_add_this_triangle(uint trianlge_id, std::vector<uint> connected_triangles, uint cluster_id);

    void set_connected_vertices();

    void print();

    void print_vertex(uint vertex_id);

    template <class T>
    bool contains(std::vector<T> v, T elem);

    bool are_vertices_only_connected_by_one_triangle(uint vertex_id1, uint vertex_id2, uint cluster_id);

    /**
    * Computes the dist of s + r, where r is positiv and the diameter of the object at s + n
    */
    bool dist_to_surface(geometry_msgs::Point s, geometry_msgs::Point r, geometry_msgs::Point n,
                         double &dist, double &diameter);

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
