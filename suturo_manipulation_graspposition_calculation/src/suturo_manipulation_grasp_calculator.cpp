//TODO: place centroid between the cluster
//TODO: centroid might not be in the polygon
//TODO: fix Pringles
//TODO: fix special case when there is only one cluster
//TODO: better sort algorithm
//TODO: remove similar grasppoints
//TODO: better error/exception handling... :(


#include "suturo_manipulation_grasp_calculator.h"

using namespace std;

using namespace ClipperLib;

using namespace suturo_manipulation;

Grasp_Calculator::Grasp_Calculator(Suturo_Manipulation_Planning_Scene_Interface *pi)
{
    pi_ = pi;
}

Grasp_Calculator::~Grasp_Calculator()
{

}

std::ostream &operator <<(std::ostream &s, const Grasp_Calculator::DoublePoint2D &p)
{
    s << "(" << p.x << "," << p.y << ")";
    return s;
}

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

geometry_msgs::Point operator+=(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return p1 + p2;
}

geometry_msgs::Point operator-=(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return p1 - p2;
}

bool operator==(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    double tolerance = 0.00001;
    return (p1.x <= p2.x + tolerance && p1.x >= p2.x - tolerance)
           && (p1.y <= p2.y + tolerance && p1.y >= p2.y - tolerance)
           && (p1.z <= p2.z + tolerance && p1.z >= p2.z - tolerance);
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

void Grasp_Calculator::addGraspPositionsZ(double d, double rotation, std::string frame_id,
        std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;

    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    //grasp from above
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, rotation);

    double grasp_hight = gripper_depth + (d > Gripper::GRIPPER_DEPTH ? d : Gripper::GRIPPER_DEPTH);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = grasp_hight;
    pre_pose = pose;
    pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);


    //grasp from below
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, rotation);

    pose.pose.position.z = 0 - grasp_hight;
    pre_pose = pose;
    pre_pose.pose.position.z -= Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

void Grasp_Calculator::addGraspPositionsX(double h, double d, double rotation, std::string frame_id,
        std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;
    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;
    double grasp_hight = gripper_depth + (d > Gripper::GRIPPER_DEPTH ? d : Gripper::GRIPPER_DEPTH);

    //grasp from the front
    pose.pose.position.x = 0 - grasp_hight;
    pose.pose.position.y = 0;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, 0);


    pre_pose = pose;
    pre_pose.pose.position.x -= Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from behind

    pose.pose.position.x = grasp_hight;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI);

    pre_pose = pose;
    pre_pose.pose.position.x += Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}

void Grasp_Calculator::addGraspPositionsY(double h, double d, double rotation, std::string frame_id,
        std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped pre_pose;
    pose.header.frame_id = frame_id;
    pre_pose.header.frame_id = frame_id;

    double grasp_hight = gripper_depth + (d > Gripper::GRIPPER_DEPTH ? d : Gripper::GRIPPER_DEPTH);
    //grasp from the left
    pose.pose.position.x = 0;
    pose.pose.position.y = grasp_hight;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, -M_PI_2);


    pre_pose = pose;
    pre_pose.pose.position.y += Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from right
    pose.pose.position.y = 0 - grasp_hight;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, M_PI_2);

    pre_pose = pose;
    pre_pose.pose.position.y -= Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);
}



bool sort_base_link_poses(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, geometry_msgs::PointStamped p)
{
    return Grasp_Calculator::get_angle(p.point, pose1.pose.position) < Grasp_Calculator::get_angle(p.point, pose2.pose.position);
}

bool sort_base_link_mesh_poses(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, geometry_msgs::PointStamped p)
{
    //~ double angle1 = Grasp_Calculator::get_angle(p.point, pose1.pose.position);
    //~ if (90 - angle1 > angle1) angle1 = 90 - angle1;
    //~ double angle2 = Grasp_Calculator::get_angle(p.point, pose2.pose.position);
    //~ if (90 - angle2 > angle2) angle2 = 90 - angle2;
    
    return pose1.pose.position.z > pose2.pose.position.z;

    //~ return angle1 < angle2;
}

int Grasp_Calculator::calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    ROS_INFO_STREAM("calculate graspposition for " << co.id);

    //test if the object is a box
    if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX)
    {
        ROS_ERROR_STREAM(co.id << " is not a box.");
        return 0;
    }

    //get object size
    double x = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
    double y = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    double z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

    //check if the object can be grasped
    if (x > Gripper::GRIPPER_MAX_POSITION &&
            y > Gripper::GRIPPER_MAX_POSITION &&
            z > Gripper::GRIPPER_MAX_POSITION)
    {
        ROS_ERROR_STREAM("Object is to big!");
        return 0;
    }

    if (x < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from above, below, left and right
        addGraspPositionsZ(z / 2, M_PI_2, co.id, poses, pre_poses, gripper_depth);

        //~ addBoxGraspPositionsY(y, 0, co.id, poses, pre_poses);
        addGraspPositionsY(0, y / 2, 0, co.id, poses, pre_poses, gripper_depth);
    }
    if (y < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from above, below, front and behind
        addGraspPositionsZ(z / 2, 0, co.id, poses, pre_poses, gripper_depth);

        addGraspPositionsX(0, x / 2, 0, co.id, poses, pre_poses, gripper_depth);
    }
    if (z < Gripper::GRIPPER_MAX_POSITION)
    {
        //object can be grasped from the left, right, front and behind
        addGraspPositionsX(0, x / 2, M_PI_2, co.id, poses, pre_poses, gripper_depth);

        addGraspPositionsY(0, y / 2, M_PI_2, co.id, poses, pre_poses, gripper_depth);
    }

    geometry_msgs::PointStamped p;
    if (!get_point_above_object(co.id, p))
    {
        return 0;
    }

    std::sort(poses.begin(), poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));
    std::sort(pre_poses.begin(), pre_poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));

    return 1;
}

bool Grasp_Calculator::get_point_above_object(std::string object_id, geometry_msgs::PointStamped &p)
{
    p.header.stamp = ros::Time(0);
    p.header.frame_id = object_id;
    try
    {
        listener_.waitForTransform(object_id, "base_link", p.header.stamp, ros::Duration(5.0));
        listener_.transformPoint("base_link", p, p);
    }
    catch (tf::TransformException t)
    {
        ROS_ERROR_STREAM(t.what());
        return false;
    }
    p.point.z += 1;
    try
    {
        listener_.waitForTransform("base_link", object_id, p.header.stamp, ros::Duration(5.0));
        listener_.transformPoint(object_id, p, p);
    }
    catch (tf::TransformException t)
    {
        ROS_ERROR_STREAM(t.what());
        return false;
    }
    return true;
}

int Grasp_Calculator::calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    ROS_INFO_STREAM("calculate graspposition for " << co.id);

    //test if object is a cylinder
    if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER)
    {
        ROS_ERROR_STREAM(co.id << " is not a cylinder.");
        return 0;
    }

    //get objectsize
    double r = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
    double h = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];

    //check if the object is too big
    if (r * 2 > Gripper::GRIPPER_MAX_POSITION)
    {
        ROS_ERROR_STREAM("Object is to big!");
        return 0;
    }

    //number of height points where we can grasp
    int grasp_pose_count = (h / cylinder_safty_dist) - 1;
    for (int i = 1; i <= grasp_pose_count; i++)
    {
        addGraspPositionsX((h / 2) - (cylinder_safty_dist * i), r, 0, co.id, poses, pre_poses, gripper_depth);
        addGraspPositionsY((h / 2) - (cylinder_safty_dist * i), r, 0, co.id, poses, pre_poses, gripper_depth);

    }

    return 1;
}

int Grasp_Calculator::calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                        std::vector<geometry_msgs::PoseStamped> &pre_poses,
                                        double gripper_depth)
{
    if (co.mesh_poses.size() != 0)
    {
        return calcMeshGraspPosition(co, poses, pre_poses, gripper_depth);
    }
    //choose the right grasppositoncalculationfunction
    switch (co.primitives[0].type)
    {
    case shape_msgs::SolidPrimitive::CYLINDER:
        return calcCylinderGraspPosition(co, poses, pre_poses, gripper_depth);
        break;

    case shape_msgs::SolidPrimitive::BOX:
        return calcBoxGraspPosition(co, poses, pre_poses, gripper_depth);
        break;

    default:
        ROS_ERROR_STREAM("Can't calculate grasppositon for objecttype: " << co.primitives[0].type);
        return 0;
        break;
    }

    ROS_INFO_STREAM("Something bad happened.");
    return 0;
}

Grasp_Calculator::DPolygon2D Grasp_Calculator::project_polygon_to_plane(Plane plane, std::vector<geometry_msgs::Point> polygon)
{
    DPolygon2D result;
    for (std::vector<geometry_msgs::Point>::iterator p = polygon.begin(); p != polygon.end(); ++p)
    {
        DoublePoint2D p2d;
        plane.project_point_to_plane(*p, p2d.x, p2d.y);
        result.push_back(p2d);
    }

    return result;
}

void Grasp_Calculator::double_polygon_to_path(DPolygon2D double_polygon, ClipperLib::Paths &int_polygon)
{
    ClipperLib::cInt factor = 100000;
    ClipperLib::Path int_poly;
    int_polygon.clear();
    for (std::vector<DoublePoint2D>::iterator p2d = double_polygon.begin(); p2d != double_polygon.end(); ++p2d)
    {
        ClipperLib::IntPoint p;
        p.X = (ClipperLib::cInt)(factor * p2d->x);
        p.Y = (ClipperLib::cInt)(factor * p2d->y);
        int_poly.push_back(p);
    }
    int_polygon.push_back(int_poly);
}

void Grasp_Calculator::path_to_double_polygon(DPolygon2D &double_polygon, ClipperLib::Path int_polygon)
{
    ClipperLib::cInt factor = 100000;
    double_polygon.clear();
    for (std::vector<IntPoint>::iterator ip = int_polygon.begin(); ip != int_polygon.end(); ++ip)
    {
        DoublePoint2D d2p;
        d2p.x = ((double)ip->X) / factor;
        d2p.y = ((double)ip->Y) / factor;
        double_polygon.push_back(d2p);
    }
}

Path Grasp_Calculator::calc_poly_centroid(Paths polygons)
{
    std::vector<IntPoint> sums;
    for (std::vector<Path>::iterator poly = polygons.begin(); poly != polygons.end(); ++poly)
    {
        IntPoint sum;
        sum.X = 0;
        sum.Y = 0;
        cInt fuck_u_Cpp = 0;
        for (std::vector<IntPoint>::iterator ip = poly->begin(); ip != poly->end(); ++ip, fuck_u_Cpp++)
        {
            sum.X += ip->X;
            sum.Y += ip->Y;
        }
        sum.X = sum.X / fuck_u_Cpp;
        sum.Y = sum.Y / fuck_u_Cpp;
        sums.push_back(sum);
    }
    return sums;
}

std::vector<geometry_msgs::Point> Grasp_Calculator::d2d_points_to_d3d_points(Plane plane, DPolygon2D polygon)
{
    std::vector<geometry_msgs::Point> result;
    for (std::vector<DoublePoint2D>::iterator d2p = polygon.begin(); d2p != polygon.end(); ++d2p)
    {
        result.push_back(d2d_point_to_d3d_point(plane, *d2p));
    }

    return result;
}

geometry_msgs::Point Grasp_Calculator::d2d_point_to_d3d_point(Plane plane, DoublePoint2D d2p)
{
    geometry_msgs::Point result;
    geometry_msgs::Point p;
    geometry_msgs::Point q;
    geometry_msgs::Point s;

    plane.get_parameter_form(s, p, q);
    result = s + d2p.x * p + d2p.y * q;

    return result;
}

geometry_msgs::Point Grasp_Calculator::cross_product(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    geometry_msgs::Point result;

    result.x = p1.y * p2.z - p1.z * p2.y;
    result.y = p1.z * p2.x - p1.x * p2.z;
    result.z = p1.x * p2.y - p1.y * p2.x;

    return result;
}

geometry_msgs::Quaternion Grasp_Calculator::get_quaternion_from_points(geometry_msgs::Point from,
        geometry_msgs::Point to,
        geometry_msgs::Point roll)
{
    geometry_msgs::Quaternion q_result;

    geometry_msgs::Point n_1;
    n_1 = to - from;
    normalize(n_1);

    geometry_msgs::Point n;
    n = roll - from;
    normalize(n);

    geometry_msgs::Point n_2;
    n_2 = n - (scalarproduct(n, n_1) * n_1);
    normalize(n_2);

    geometry_msgs::Point n_3;
    n_3 = cross_product(n_1, n_2);
    normalize(n_3);

    tf::Matrix3x3 rotation_matrix(n_1.x, n_2.x, n_3.x,
                                  n_1.y, n_2.y, n_3.y,
                                  n_1.z, n_2.z, n_3.z);
    tf::Quaternion q;

    rotation_matrix.getRotation(q);
    tf::quaternionTFToMsg(q, q_result);

    return q_result;
}

bool Grasp_Calculator::get_grasp_point(Plane plane, geometry_msgs::Point m, double d, double alpha,
                                       geometry_msgs::PoseStamped &grasp_pose,
                                       geometry_msgs::PoseStamped &pre_grasp_pose,
                                       Mesh meshi)
{
    DoublePoint2D p2d;
    p2d.x = sin(alpha) * d;
    p2d.y = cos(alpha) * d;
    geometry_msgs::Point r = d2d_point_to_d3d_point(plane, p2d);

    geometry_msgs::Point c;
    c = m + plane.get_normal();

    // r = r + m;

    grasp_pose.pose.position = r + m;

    grasp_pose.pose.orientation = get_quaternion_from_points(grasp_pose.pose.position, m, c);

    double dist = 0;
    double diameter = 0;
    meshi.dist_to_surface(m, r, plane.get_normal(), dist, diameter);
    // ROS_INFO_STREAM(diameter);
    if (diameter >= Gripper::GRIPPER_MAX_POSITION) return false;
    double grasp_hight = d + (dist > Gripper::GRIPPER_DEPTH ? dist : Gripper::GRIPPER_DEPTH);
    // ROS_INFO_STREAM("dist " << d);

    pre_grasp_pose.pose.orientation = grasp_pose.pose.orientation;
    normalize(r);
    grasp_pose.pose.position = m + grasp_hight * r;
    pre_grasp_pose.pose.position = m + ((double)(grasp_hight + Gripper::GRIPPER_DEPTH) ) * r;
    return true;
}

int Grasp_Calculator::calcMeshGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    ros::Time t = ros::Time::now();
    string gripper_group = gripper_depth == Gripper::R_GRIPPER_PALM_LENGTH ?
                           Gripper::get_r_group_name() : Gripper::get_l_group_name();


    std::vector<geometry_msgs::Point> c;
    suturo_manipulation::Mesh meshi(co);

    // 2. search for cluster with opposite normal - O(|Cluster|²)
    std::vector< std::pair<uint, uint> > opposite_cluster = meshi.get_opposite_cluster();
    int h = 0;
    ROS_INFO_STREAM("opposite cluster size: " << opposite_cluster.size());
    for (int i = 0; i < opposite_cluster.size(); i++)
    {
        // 3. project both cluster on a plain - o(|vertices|)?
        suturo_manipulation::Plane plane = meshi.get_plane(opposite_cluster[i].first, opposite_cluster[i].second);
        plane.orthonormalize();

        std::vector<geometry_msgs::Point> p1 = meshi.get_polygon(opposite_cluster[i].first);
        std::vector<geometry_msgs::Point> p2 = meshi.get_polygon(opposite_cluster[i].second);
        if (i == 0)
        {
            pi_->publishMarkerLine(co.id, p1, 0);
            pi_->publishMarkerLine(co.id, p2, 1000);

        }

        DPolygon2D dpolygon1 = project_polygon_to_plane(plane, p1);
        DPolygon2D dpolygon2 = project_polygon_to_plane(plane, p2);

        ClipperLib::Paths polygon1(1);
        ClipperLib::Paths polygon2(1);
        double_polygon_to_path(dpolygon1, polygon1);
        double_polygon_to_path(dpolygon2, polygon2);


        // 4. calc polygon intersection - O(|vertices|²)?
        //     http://www.integis.ch/documents/ISem_Opprecht_Overlay_2002-02-28.pdf
        //     http://www.cs.man.ac.uk/~toby/alan/software/gpc.html
        ClipperLib::Clipper clpr;
        clpr.AddPaths(polygon1, ClipperLib::ptSubject, true);
        clpr.AddPaths(polygon2, ClipperLib::ptClip, true);
        ClipperLib::Paths solution;
        clpr.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);


        // 5. calc centriod - O(|vertices|)
        Path int_centroids = calc_poly_centroid(solution);

        DPolygon2D double_centroids;
        path_to_double_polygon(double_centroids, int_centroids);
        std::vector<geometry_msgs::Point> centroids = d2d_points_to_d3d_points(plane, double_centroids);
        for (std::vector<geometry_msgs::Point>::iterator i = centroids.begin(); i != centroids.end(); ++i)
        {
            c.push_back(*i);
        }

        //ggf 5.2 check dist to cluster
        // http://www.uninformativ.de/bin/RaytracingSchnitttests-76a577a-CC-BY.pdf


        // 6. add grasppose pointing towards centriod with differen angles - const

        for (std::vector<geometry_msgs::Point>::iterator i = centroids.begin(); i != centroids.end(); ++i)
        {
            for (double a = 0; a < 2 * M_PI; a += (M_PI / 2))
            {
                geometry_msgs::PoseStamped temp_grasp_pose;
                geometry_msgs::PoseStamped temp_pre_grasp_pose;
                temp_grasp_pose.header.frame_id = co.id;
                temp_pre_grasp_pose.header.frame_id = co.id;

                if (!get_grasp_point(plane, *i, gripper_depth , a, temp_grasp_pose, temp_pre_grasp_pose, meshi))
                    continue;

                // h++;
                // 7. use moveit to test poses - O(much)???
                // if (!pi_->check_group_object_collision(gripper_group, temp_grasp_pose, co))
                // {
                // ROS_INFO_STREAM("collision!" );
                // ros::WallDuration(0.5).sleep();
                // ROS_WARN_STREAM("centroid " << *i);
                poses.push_back(temp_grasp_pose);
                pre_poses.push_back(temp_pre_grasp_pose);


                // }
            }
        }
    }
    pi_->publishMarkerPoints(co.id, c);
    if (poses.empty())
    {
        ROS_WARN_STREAM("No Grasppositions found for: " << co.id);
        return 0;
    }

    //sort
    geometry_msgs::PointStamped p;
    if (!get_point_above_object(co.id, p)) return 0;
    std::sort(poses.begin(), poses.end(), boost::bind(sort_base_link_mesh_poses, _1, _2, p));
    std::sort(pre_poses.begin(), pre_poses.end(), boost::bind(sort_base_link_mesh_poses, _1, _2, p));

    ROS_INFO_STREAM("poses:" << poses.size());
    for (int i = 0; i < poses.size() ; ++i)
    {
        pi_->publishMarker(poses[i], i);
        pi_->publishMarker(pre_poses[i], i + poses.size());
    }
    ROS_INFO_STREAM((ros::Time::now() - t));
    return 1;
}