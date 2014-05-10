#include "suturo_manipulation_mesh.h"
#include <geometric_shapes/shape_operations.h>
#include <shape_tools/solid_primitive_dims.h>

using namespace suturo_manipulation;

geometry_msgs::Point operator+(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    geometry_msgs::Point r;
    r.x = p1.x + p2.x;
    r.y = p1.y + p2.y;
    r.z = p1.z + p2.z;
    return r;
}

geometry_msgs::Point operator-(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    geometry_msgs::Point r;
    r.x = p1.x - p2.x;
    r.y = p1.y - p2.y;
    r.z = p1.z - p2.z;
    return r;
}

geometry_msgs::Point operator*(const double &d, const geometry_msgs::Point &p1)
{
    geometry_msgs::Point r;
    r.x = d * p1.x;
    r.y = d * p1.y;
    r.z = d * p1.z;
    return r;
}

double scalarproduct(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return (p1.x * p2.x) +
           (p1.y * p2.y) +
           (p1.z * p2.z);
}

bool normalize(geometry_msgs::Point &p)
{
    if (p.x == 0 && p.y == 0 && p.z == 0) return false;
    double a = 1 / sqrt((p.x * p.x + p.y * p.y + p.z * p.z));
    p.x = p.x * a;
    p.y = p.y * a;
    p.z = p.z * a;
    return true;
}

//Plane------------------------------------------------------------------------------------------------------------------------------
Plane::Plane(geometry_msgs::Point normal,
             geometry_msgs::Point support_vector,
             geometry_msgs::Point first_parameter,
             geometry_msgs::Point second_parameter)
{
    support_vector_ = support_vector;

    first_parameter_ = first_parameter;

    second_parameter_ = second_parameter;

    normal_ = normal;

    d = scalarproduct(normal_, support_vector_);
}

bool Plane::orthonormalize()
{
    normalize(first_parameter_);

    normalize(second_parameter_);

    second_parameter_ = second_parameter_ -
                        (scalarproduct(second_parameter_, first_parameter_) * first_parameter_);
    normalize(second_parameter_);

    normalize(normal_);

    d = scalarproduct(normal_, support_vector_);

    return pre_compute();
}

bool Plane::pre_compute()
{
    //for 2d point calculation

    double qq = scalarproduct(second_parameter_, second_parameter_);
    double pp = scalarproduct(first_parameter_, first_parameter_);
    double pq = scalarproduct(first_parameter_, second_parameter_);

    double denominator = qq * pp - pq * pq;

    if (denominator == 0) return false;

    u_a = ((qq / denominator) * first_parameter_) - ((pq / denominator) * second_parameter_);
    u_b = ((pp / denominator) * second_parameter_) - ((pq / denominator) * first_parameter_);
    return true;
}

bool Plane::get_point_of_intersection(geometry_msgs::Point &result, geometry_msgs::Point s, geometry_msgs::Point r, double &lamda)
{
    double temp = scalarproduct(normal_, r);

    if (temp == 0)
        return false;

    lamda = ((d - scalarproduct(normal_, s)) / temp );

    result = s + lamda * r ;
    return true;
}

//x = support_vector_ + a*first_parameter_ + b*second_parameter_
bool Plane::d3d_point_to_d2d_point(double &a, double &b, geometry_msgs::Point x)
{
    geometry_msgs::Point y = x - support_vector_;

    a = scalarproduct(y, u_a);

    b = scalarproduct(y, u_b);

    return true;
}

void Plane::get_coordinate_form(double &a, double &b, double &c, double &d)
{
    a = normal_.x;
    b = normal_.y;
    c = normal_.z;
    d = d;
};

void Plane::project_point_to_plane(geometry_msgs::Point p, double &x, double &y)
{
    geometry_msgs::Point r;
    double ignore = 0;
    get_point_of_intersection(r, p, get_normal(), ignore);
    d3d_point_to_d2d_point(x, y, r);
}

bool Plane::dist_to_plane_triangle(geometry_msgs::Point s, geometry_msgs::Point r, double &dist)
{
    geometry_msgs::Point p;
    double lamda = 0;
    if (!get_point_of_intersection(p, s, r, lamda) || lamda < 0)
    {

        return false;
    }
    double a;
    double b;
    d3d_point_to_d2d_point(a, b, p);

    if (a >= 0 && b >= 0 && (1 - a - b) >= 0)
    {
        geometry_msgs::Point s_to_p = p - s;
        dist = sqrt(scalarproduct(s_to_p, s_to_p));
        return true;
    }
    return false;
}

bool Plane::dist_to_plane_triangle2(geometry_msgs::Point s, geometry_msgs::Point r, double &dist)
{
    geometry_msgs::Point p;
    double lamda = 0;
    if (!get_point_of_intersection(p, s, r, lamda))
    {
        return false;
    }
    double a;
    double b;
    d3d_point_to_d2d_point(a, b, p);

    if (a >= 0 && b >= 0 && (1 - a - b) >= 0)
    {
        geometry_msgs::Point s_to_p = p - s;
        dist = sqrt(scalarproduct(s_to_p, s_to_p));
        return true;
    }
    return false;
}

//Plane------------------------------------------------------------------------------------------------------------------------------

void Triangle::calc_plane(std::vector<Vertex> vertices1)
{
    geometry_msgs::Point first_parameter;
    geometry_msgs::Point second_parameter;

    first_parameter = vertices1[vertices[1]].position - vertices1[vertices[0]].position;
    second_parameter = vertices1[vertices[2]].position - vertices1[vertices[0]].position;
    plane_ = new Plane(normal, vertices1[vertices[0]].position, first_parameter, second_parameter);
    plane_->pre_compute();
}


Mesh::Mesh(shapes::Mesh *mesh)
{
    // 1. build simple polygon cluster - O(|Triangle|²)?
    // 1.1. take random triangle
    // 1.2. search for triangle with similar normal and with >=2 contacts
    // (1.2.1 update representative triangle)
    // 1.3. repeat until no new trianlge added
    // (1.4 remove small clusters)
    build_cluster(mesh);
}

Mesh::Mesh(moveit_msgs::CollisionObject co)
{
    if (co.meshes.empty()) ROS_ERROR_STREAM("invalid mesh collision object");
    shapes::Mesh *mesh = (shapes::Mesh *)shapes::constructShapeFromMsg(co.meshes[0]);
    build_cluster(mesh);
}

Mesh::~Mesh()
{

}

std::vector< std::pair<uint, uint> > Mesh::get_opposite_cluster()
{
    if (opposite_cluster.empty())
    {
        double threshold = 0.2;
        for (int i = 0; i < clusters.size(); i++)
        {
            for (int j = i + 1; j < clusters.size(); j++)
            {
                // ROS_INFO_STREAM("comp i= " << i << " j= " << j);
                if (M_PI - get_angle(clusters[i].normal, clusters[j].normal) < threshold)
                {
                    std::pair<uint, uint> c_pair(i, j);
                    opposite_cluster.push_back(c_pair);
                }
            }
        }
    }
    return opposite_cluster;
}



Plane Mesh::get_plane(uint cluster_id1, uint cluster_id2)
{
    geometry_msgs::Point normal = get_cluster(cluster_id1).normal;
    geometry_msgs::Point normal2 = get_cluster(cluster_id2).normal;
    geometry_msgs::Point plane_normal;
    plane_normal.x = normal.x - normal2.x;
    plane_normal.y = normal.y - normal2.y;
    plane_normal.z = normal.z - normal2.z;

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    if (plane_normal.x == 0 && plane_normal.y == 0)
    {
        p1.x = -plane_normal.z;
        p1.y = 0;
        p1.z = plane_normal.x;

        p2.x = 0;
        p2.y = -plane_normal.z;
        p2.z = plane_normal.y;

    }
    else if (plane_normal.z == 0 && plane_normal.y == 0)
    {
        p1.x = -plane_normal.z;
        p1.y = 0;
        p1.z = plane_normal.x;

        p2.x = -plane_normal.y;
        p2.y = plane_normal.x;
        p2.z = 0;
    }
    else
    {
        p1.x = 0;
        p1.y = -plane_normal.z;
        p1.z = plane_normal.y;

        p2.x = -plane_normal.y;
        p2.y = plane_normal.x;
        p2.z = 0;
    }

    if (!normalize(p1)) ROS_ERROR_STREAM("!!!!");
    if (!normalize(p2)) ROS_ERROR_STREAM("!!!!");

    geometry_msgs::Point o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    Plane plane(plane_normal, o, p1, p2);

    return plane;
}



std::vector<geometry_msgs::Point> Mesh::create_polygon(uint cluster_id)
{
    std::vector<uint> poly;
    //1. find first bondary point
    for (int i = 0; i < clusters[cluster_id].vertices.size(); i++)
    {
        uint vertex_id = clusters[cluster_id].vertices[i];
        // ROS_INFO_STREAM("vertex_id : " << vertex_id);
        // ROS_INFO_STREAM("vertex cluster size : " << vertices[vertex_id].clusters.size());
        if (is_vertex_boundary_point(vertex_id, cluster_id))
        {
            //2. add to poly
            poly.push_back(vertex_id);
            break;
        }
    }
    // ROS_INFO_STREAM("ok");
    //3. get next BP
    std::vector<uint> next_vertex_ids;
    // ROS_INFO_STREAM("ok : " << poly[0]);
    for (get_next_boundary_point(poly[0], cluster_id, next_vertex_ids);
            ;//!(contains(poly, next_vertex_ids[0]) && contains(poly, next_vertex_ids[1]));
            get_next_boundary_point(poly[(poly.size() - 1)], cluster_id, next_vertex_ids))
    {
        // ROS_INFO_STREAM("ok : " << poly[(poly.size() - 1)]);
        if (next_vertex_ids.size() > 2)
            // {
            ROS_ERROR_STREAM("next vertex size to big!!!" << next_vertex_ids.size());
        // else ROS_INFO_STREAM("nope");
        // for (int i = 0; i < next_vertex_ids.size(); ++i)
        // {
        //     print_vertex(next_vertex_ids[i]);
        // }
        // }
        if (!contains(poly, next_vertex_ids[0]))
        {
            poly.push_back(next_vertex_ids[0]);
        }
        else if (!contains(poly, next_vertex_ids[1]))
        {
            poly.push_back(next_vertex_ids[1]);
        }
        //4. if both BP in poly finish
        else
        {
            // ROS_INFO_STREAM("polygon closed!!! " << next_vertex_ids.size());
            break;

        }
    }
    std::vector<geometry_msgs::Point> result;
    for (int i = 0; i < poly.size(); ++i)
    {
        result.push_back(vertices[poly[i]].position);
    }
    return result;
}

void Mesh::get_next_boundary_point(uint vertex_id, uint cluster_id, std::vector<uint> &next_vertex_ids)
{
    // Vertex np1 = vertices[vertex_id].triangles[i];
    next_vertex_ids.clear();
    for (std::vector<uint>::iterator n_v = vertices[vertex_id].connected_vertices.begin();
            n_v != vertices[vertex_id].connected_vertices.end();
            ++n_v)
    {
        // ROS_INFO_STREAM("id: " << *n_v << " BP? " << is_vertex_boundary_point(*n_v, cluster_id) << " muh " << are_vertices_only_connected_by_one_triangle(*n_v, vertex_id, cluster_id));
        if (is_vertex_boundary_point(*n_v, cluster_id)
                && are_vertices_only_connected_by_one_triangle(*n_v, vertex_id, cluster_id))
        {
            next_vertex_ids.push_back(*n_v);
        }
    }

}

bool Mesh::are_vertices_only_connected_by_one_triangle(uint vertex_id1, uint vertex_id2, uint cluster_id)
{
    std::vector<uint> v;
    std::set_intersection(vertices[vertex_id1].triangles.begin(),
                          vertices[vertex_id1].triangles.end(),
                          vertices[vertex_id2].triangles.begin(),
                          vertices[vertex_id2].triangles.end(),
                          back_inserter(v));
    int c = v.size();
    for (int i = 0; i < v.size(); ++i)
    {
        if (!contains(clusters[cluster_id].triangles, v[i]) )
            c--;
    }
    return c == 1;
}

void Mesh::add_triangle_to_cluster(uint cluster_id, uint triangle_id)
{
    uint num_triangles = clusters[cluster_id].triangles.size();
    clusters[cluster_id].triangles.push_back(triangle_id);
    if (!contains(triangles[triangle_id].clusters, cluster_id))
    {
        // triangle doenst has cluster in his list
        triangles[triangle_id].clusters.push_back(cluster_id);
    }
    //add new vertices to cluster
    for (std::vector<uint>::iterator vertex_id = triangles[triangle_id].vertices.begin(); vertex_id != triangles[triangle_id].vertices.end(); ++vertex_id)
    {
        if (!contains(clusters[cluster_id].vertices, *vertex_id))
        {
            clusters[cluster_id].vertices.push_back(*vertex_id);
            vertices[*vertex_id].clusters.push_back(cluster_id);
        }
    }
    //update normal
    geometry_msgs::Point p;
    p.x = clusters[cluster_id].normal.x * num_triangles;
    p.y = clusters[cluster_id].normal.y * num_triangles;
    p.z = clusters[cluster_id].normal.z * num_triangles;

    clusters[cluster_id].normal.x = (p.x + triangles[triangle_id].normal.x) / (num_triangles + 1);
    clusters[cluster_id].normal.y = (p.y + triangles[triangle_id].normal.y) / (num_triangles + 1);
    clusters[cluster_id].normal.z = (p.z + triangles[triangle_id].normal.z) / (num_triangles + 1);

}

void Mesh::build_cluster(shapes::Mesh *mesh)
{
    double threshold = 0.6;

    //create better useable datatypes
    // ROS_ERROR_STREAM("VERTEX : " << mesh->vertex_count);
    for (int i = 0; i < mesh->vertex_count; i++)
    {
        Vertex v;
        geometry_msgs::Point p;
        p.x = mesh->vertices[3 * i];
        p.y = mesh->vertices[(3 * i) + 1];
        p.z = mesh->vertices[(3 * i) + 2];
        v.position = p;
        vertices.push_back(v);
    }

    mesh->computeTriangleNormals();

    std::vector< std::pair<Triangle, uint> > mesh_triangles;
    // ROS_ERROR_STREAM("TRIANGLE : " << mesh->triangle_count);
    for (int i = 0; i < mesh->triangle_count; i++)
    {
        Triangle t;
        uint vertex_id = mesh->triangles[(i * 3)];
        t.vertices.push_back(vertex_id);
        //add triangle to vertex
        vertices[vertex_id].triangles.push_back(i);

        vertex_id = mesh->triangles[(i * 3) + 1];
        t.vertices.push_back(vertex_id);
        vertices[vertex_id].triangles.push_back(i);

        vertex_id = mesh->triangles[(i * 3) + 2];
        t.vertices.push_back(vertex_id);
        // ROS_ERROR_STREAM("vertex_id: " << mesh->triangles[(i * 3)] << " " << mesh->triangles[(i * 3)+1] << " " <<
        //     mesh->triangles[(i * 3)+2]);
        vertices[vertex_id].triangles.push_back(i);

        //sort for later intersection
        sort(t.vertices.begin(), t.vertices.end());
        t.normal.x = mesh->triangle_normals[i * 3];
        t.normal.y = mesh->triangle_normals[(i * 3) + 1];
        t.normal.z = mesh->triangle_normals[(i * 3) + 2];

        t.calc_plane(vertices);

        triangles.push_back(t);

        std::pair<Triangle, uint> pair(t, i);
        mesh_triangles.push_back(pair);
    }

    uint cluster_id = 0;
    while (mesh_triangles.size() > 0)
    {
        // ROS_INFO_STREAM(mesh_triangles.size());
        Cluster c;
        // c.normal = mesh_triangles[0].first.normal;
        clusters.push_back(c);
        add_triangle_to_cluster(cluster_id, mesh_triangles[0].second);
        mesh_triangles.erase(mesh_triangles.begin());
        for (int i = 0; i < mesh_triangles.size();)
        {
            //has the triangle a similar normal?
            double a = get_angle(clusters[cluster_id].normal, mesh_triangles[i].first.normal);
            if (a < threshold)
            {
                //check for two connections
                std::vector<uint> v;
                std::vector<uint> connected_triangles;
                for (int trianlge_id = 0; trianlge_id < clusters[cluster_id].triangles.size(); trianlge_id++)
                {
                    std::set_intersection(get_triangle(clusters[cluster_id].triangles[trianlge_id]).vertices.begin(),
                                          get_triangle(clusters[cluster_id].triangles[trianlge_id]).vertices.end(),
                                          mesh_triangles[i].first.vertices.begin(),
                                          mesh_triangles[i].first.vertices.end(),
                                          back_inserter(v));
                    if (v.size() >= 2)
                        connected_triangles.push_back(trianlge_id);
                    // break;
                }
                if (u_have_to_add_this_triangle(mesh_triangles[i].second, connected_triangles, cluster_id))
                {
                    //triangle is connected to 2 triangles in cluster
                    add_triangle_to_cluster(cluster_id, mesh_triangles[i].second);
                    mesh_triangles.erase(mesh_triangles.begin() + i);
                    //start over, because new triangles could be connected
                    i = 0;
                }
                else i++;
            }
            else i++;
        }
        cluster_id++;
    }
    set_connected_vertices();
}

void Mesh::set_connected_vertices()
{
    // for (std::vector<Vertex>::iterator v = vertices.begin(); v != vertices.end(); ++v)
    // {
    for (uint v_id = 0; v_id < vertices.size(); ++v_id)
    {
        // Vertex v = vertices[v_id];
        for (std::vector<uint>::iterator t = vertices[v_id].triangles.begin(); t != vertices[v_id].triangles.end(); ++t)
        {
            for (std::vector<uint>::iterator v2 = triangles[*t].vertices.begin();
                    v2 != triangles[*t].vertices.end();
                    ++v2)
            {
                // ROS_INFO_STREAM(*v2);
                if (v_id != *v2 && !contains(vertices[v_id].connected_vertices, *v2))
                    vertices[v_id].connected_vertices.push_back(*v2);
            }
        }
    }
    // }
}

bool Mesh::u_have_to_add_this_triangle(uint trianlge_id, std::vector<uint> connected_triangles, uint cluster_id)
{
    //more then 1 triangle conntected
    return connected_triangles.size() >= 2
           //only 1 triangle conntected
           || (connected_triangles.size() == 1
               //and at least 1 vertex isn't already in the cluster
               && !(is_vertex_in_cluster(triangles[trianlge_id].vertices[0], cluster_id)
                    && is_vertex_in_cluster(triangles[trianlge_id].vertices[1], cluster_id)
                    && is_vertex_in_cluster(triangles[trianlge_id].vertices[2], cluster_id)));
}


bool Mesh::is_vertex_in_cluster(uint vertex_id, uint cluster_id)
{
    return contains(vertices[vertex_id].clusters, cluster_id);
}


template <class T>
bool Mesh::contains(std::vector<T> v, T elem)
{
    return std::find(v.begin(), v.end(), elem) != v.end();
}

bool Mesh::is_vertex_boundary_point(uint vertex_id, uint cluster_id)
{
    //TODO: sonderfall, mesh is ein einziges cluster
    //Is the vertex part of more then one cluster and is he part of this cluster?
    return vertices[vertex_id].clusters.size() > 1
           && contains(vertices[vertex_id].clusters, cluster_id);
}

bool Mesh::dist_to_surface(geometry_msgs::Point s, geometry_msgs::Point r, geometry_msgs::Point n, double &dist, double &diameter)
{
    //setzt vorraus, dass dist niemals 0 ist, aber passt schon :)
    dist = 0;
    diameter = 0;
    double diameter1 = 0;
    double diameter2 = 0;
    for (int t_id = 0; t_id < triangles.size(); ++t_id)
    {
        if (dist == 0)
            triangles[t_id].plane_->dist_to_plane_triangle(s, r, dist);
        
        if (diameter1 == 0 && triangles[t_id].plane_->dist_to_plane_triangle2(s, n, diameter1))
        {
            continue;
        }
        else if (diameter1 != 0 && diameter2 == 0 && triangles[t_id].plane_->dist_to_plane_triangle2(s, n, diameter2))
        {
            diameter = diameter1 + diameter2;
        }
        if  (dist != 0 && diameter != 0)
        {
            return true;
        }


    }
    return false;
}

Cluster Mesh::get_cluster(uint id)
{
    return clusters[id];
}

Triangle Mesh::get_triangle(uint id)
{
    return triangles[id];
}

Vertex Mesh::get_vertex(uint id)
{
    return vertices[id];
}

void Mesh::add_cluster(Cluster c)
{
    clusters.push_back(c);
}


void Mesh::add_triangle(Triangle t)
{
    triangles.push_back(t);
}

void Mesh::add_vertex(Vertex v)
{
    vertices.push_back(v);
}


std::vector<Cluster> Mesh::get_clusters()
{
    return clusters;
}


std::vector<Triangle> Mesh::get_triangles()
{
    return triangles;
}


std::vector<Vertex> Mesh::get_vertices()
{
    return vertices;
}

void Mesh::print_vertex(uint vertex_id)
{
    // ROS_INFO_STREAM("Vertex ID: " << vertex_id);
    // for (int i = 0; i < vertices[vertex_id].triangles.size(); ++i)
    // {
    //     ROS_INFO_STREAM(triangles[i].toString(clusters, vertices));
    //     for (int j = 0; j < triangles[i].vertices.size(); ++j)
    //     {
    //         ROS_INFO_STREAM(triangles[i].vertices[j]);
    //     }
    //     ROS_INFO_STREAM("connected_vertices");
    //     for (int j = 0; j < vertices[i].connected_vertices.size(); ++j)
    //     {
    //         ROS_INFO_STREAM(vertices[i].connected_vertices[j]);
    //     }

    // }
    // ROS_INFO_STREAM(" ");
}

void Mesh::print()
{
    //print cluster
    ROS_INFO_STREAM("Cluster size: " << clusters.size());
    for (int i = 0; i < clusters.size(); ++i)
    {

        ROS_INFO_STREAM("Cluster ID: " << i);
        ROS_INFO_STREAM("Cluster triangle size: " << clusters[i].triangles.size());
        ROS_INFO_STREAM("Cluster vertices size: " << clusters[i].vertices.size());
        ROS_INFO_STREAM("Cluster normal: " << clusters[i].normal);
    }

    //print triangles
    ROS_INFO_STREAM("Triangles size: " << triangles.size());
    // for (int i = 0; i < triangles.size(); ++i)
    // {

    //     ROS_INFO_STREAM("Triangle ID: " << i);
    //     ROS_INFO_STREAM("Triangle cluster size: " << triangles[i].clusters.size());
    //     ROS_INFO_STREAM("Triangle vertices size: " << triangles[i].vertices.size());
    //     ROS_INFO_STREAM("Triangle normal: " << triangles[i].normal);
    // }

    //print vertices
    ROS_INFO_STREAM("Vertices size: " << vertices.size());
    // for (int i = 0; i < vertices.size(); ++i)
    // {

    //     ROS_INFO_STREAM("Vertices ID: " << i);
    //     ROS_INFO_STREAM("Vertices triangle size: " << vertices[i].triangles.size());
    //     ROS_INFO_STREAM("Vertices connected vertices size: " << vertices[i].connected_vertices.size());
    //     ROS_INFO_STREAM("Vertices cluster: " << vertices[i].clusters.size());
    // }
}
