//TODO: place centroid between the cluster
//TODO: centroid might not be in the polygon
//TODO: fix special case when there is only one cluster
//TODO: better sort algorithm


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
    double angle1 = Grasp_Calculator::get_angle(p.point, pose1.pose.position);
    if (90 - angle1 > angle1) angle1 = 90 - angle1;
    double angle2 = Grasp_Calculator::get_angle(p.point, pose2.pose.position);
    if (90 - angle2 > angle2) angle2 = 90 - angle2;

    return angle1 < angle2;
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

    geometry_msgs::PointStamped p = get_point_above_object(co.id);

    std::sort(poses.begin(), poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));
    std::sort(pre_poses.begin(), pre_poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));

    return 1;
}

geometry_msgs::PointStamped Grasp_Calculator::get_point_above_object(std::string object_id)
{
    geometry_msgs::PointStamped p;
    p.header.stamp = ros::Time();
    p.header.frame_id = object_id;
    try
    {
        listener_.transformPoint("base_link", p, p);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
    }
    p.point.z += 1;
    try
    {
        listener_.transformPoint(object_id, p, p);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
    }
    return p;
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

void Grasp_Calculator::transform_poses(std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses)
{
    try
    {
        //transform pose to base_link
        for (int i = 0; i < poses.size(); i++)
        {
            listener_.transformPose(frame_id, poses[i], poses[i]);

        }
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
    }
}

geometry_msgs::Point Grasp_Calculator::get_point_of_intersection(suturo_manipulation::Plane plane, geometry_msgs::Point p)
{
    geometry_msgs::Point r;
    geometry_msgs::Point n = plane.get_normal();

    double lamda = (- (p.x * n.x + p.y * n.y + p.z * n.z) /
                    (n.x * n.x + n.y * n.y + n.z * n.z));

    r.x = p.x + lamda * n.x;
    r.y = p.y + lamda * n.y;
    r.z = p.z + lamda * n.z;
    return r;
}

double get_b(double x_n, double q_n, double p_n, double a)
{
    double b = (x_n - a * p_n) / q_n;
    return b;
}

double get_a(double x_n, double q_n, double p_n, double b)
{
    double a = (x_n - b * q_n) / p_n;
    return a;
}

double get_a_long_m(double x_m, double x_n, double q_m, double q_n, double p_m, double p_n)
{
    double a = (x_m * q_n - q_m * x_n) /
               (p_m * q_n - p_n * q_m);
    return a;
}

int get_a_b(double p_n, double q_n, double x_n, double &a, double &b)
{
    if ( p_n != 0 && q_n == 0)
    {
        //first equotion contains 0
        a = x_n / p_n;
        return 2;
    }
    if (p_n == 0 && q_n != 0)
    {
        b = x_n / q_n;
        return 3;
    }
    if (p_n == 0 && q_n == 0)
    {
        return 5;
    }
    return 7;
}

bool solve(double x_n, double x_m, double p_n, double p_m, double q_n, double q_m, double &a, double &b)
{
    int n_gleichung = get_a_b(p_n, q_n, x_n, a, b);
    int m_gleichung = get_a_b(p_m, q_m, x_m, a, b);
    if (n_gleichung * m_gleichung == 6)
    {
        //a und b mit ersten beiden gleichungen gesetzt
        //fertig
        return true;
    }
    if (((n_gleichung * m_gleichung) % 5 == 0)
            || ((n_gleichung * m_gleichung) % 4 == 0)
            || ((n_gleichung * m_gleichung) % 9 == 0))
    {
        //eine gleichung hat 2x 0
        return false;
    }
    if (n_gleichung % 2 == 0)
    {
        //n gleichung hat a ergeben
        //a ist fertig
        b = get_b(x_m, q_m, p_m, a);
        return true;
    }
    if (m_gleichung % 2 == 0)
    {
        //m gleichung hat a ergeben
        //a ist fertig
        b = get_b(x_n, q_n, p_n, a);
        return true;
    }
    if (n_gleichung % 3 == 0)
    {
        //n gleichung hat a ergeben
        //b ist fertig
        a = get_a(x_m, q_m, p_m, b);
        return true;
    }
    if (m_gleichung % 3 == 0)
    {
        //n gleichung hat a ergeben
        //b ist fertig
        a = get_a(x_n, q_n, p_n, b);
        return true;
    }
    if (n_gleichung * m_gleichung % 49 == 0)
    {
        //keiner fertig und keine nullen
        //a berechnen, indem n gleichung nach b umgestellt und in m gleichung eingesetzt wird
        a = get_a_long_m(x_n, x_m, q_n, q_m, p_n, p_m);
        //a in n gleichung einsetzten

        b = get_b(x_n, q_n, p_n, a);
        return true;
    }

    return false;
}

bool Grasp_Calculator::d3d_point_to_d2d_point(DoublePoint2D &p2d, geometry_msgs::Point &p, geometry_msgs::Point &q, geometry_msgs::Point &r)
{
    return solve(r.x, r.y, p.x, p.y, q.x, q.y, p2d.x, p2d.y)
           || solve(r.x, r.z, p.x, p.z, q.x, q.z, p2d.x, p2d.y)
           || solve(r.y, r.z, p.y, p.z, q.y, q.z, p2d.x, p2d.y);
}

bool operator==(const geometry_msgs::Point p1, const geometry_msgs::Point p2)
{
    double tolerance = 0.00001;
    return (p1.x <= p2.x + tolerance && p1.x >= p2.x - tolerance)
           && (p1.y <= p2.y + tolerance && p1.y >= p2.y - tolerance)
           && (p1.z <= p2.z + tolerance && p1.z >= p2.z - tolerance);
}

Grasp_Calculator::DPolygon2D Grasp_Calculator::project_polygon_to_plane(suturo_manipulation::Plane plane, std::vector<geometry_msgs::Point> polygon)
{
    std::vector<geometry_msgs::Point> plane_poly;
    for (std::vector<geometry_msgs::Point>::iterator p = polygon.begin(); p != polygon.end(); ++p)
    {
        plane_poly.push_back(get_point_of_intersection(plane, *p));
    }
    // for (int i = 0; i < plane_poly.size(); ++i)
    // {
    //     ROS_INFO_STREAM(plane_poly[i]);
    // }
    // ROS_ERROR_STREAM(" nicht");


    DPolygon2D result;
    geometry_msgs::Point p = plane.get_parameter_form().first;
    geometry_msgs::Point q = plane.get_parameter_form().second;
    // ROS_INFO_STREAM("planeparameter1  x= " << a.x << " y= " << a.y << " z= " << a.z);
    // ROS_INFO_STREAM("planeparameter2  x= " << b.x << " y= " << b.y << " z= " << b.z);
    for (std::vector<geometry_msgs::Point>::iterator r = plane_poly.begin(); r != plane_poly.end(); ++r)
    {
        DoublePoint2D p2d;
        d3d_point_to_d2d_point(p2d, p, q, *r);
        geometry_msgs::Point p = d2d_point_to_d3d_point(plane, p2d);
        if (!(*r == p)) ROS_INFO_STREAM(*r << "     " << p << std::endl);

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
        // ROS_INFO_STREAM("p.x  " << p.X << " p2d " << p2d->x);
        p.Y = (ClipperLib::cInt)(factor * p2d->y);
        // ROS_INFO_STREAM("p.y  " << p.Y << " p2d " << p2d->y);
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
    // ROS_ERROR_STREAM("???");
    // ROS_INFO_STREAM(polygons.size());
    for (std::vector<Path>::iterator poly = polygons.begin(); poly != polygons.end(); ++poly)
    {
        IntPoint sum;
        sum.X = 0;
        sum.Y = 0;
        cInt fuck_u_C = 0;
        for (std::vector<IntPoint>::iterator ip = poly->begin(); ip != poly->end(); ++ip, fuck_u_C++)
        {
            // ROS_WARN_STREAM("ip " << *ip);
            sum.X += ip->X;
            sum.Y += ip->Y;
            // ROS_ERROR_STREAM("sum " << sum);
        }
        // ROS_INFO_STREAM((sum.X ));
        sum.X = sum.X / fuck_u_C;
        // ROS_INFO_STREAM((sum.X / fuck_u_C));
        sum.Y = sum.Y / fuck_u_C;
        // ROS_INFO_STREAM((sum.X ));
        // ROS_ERROR_STREAM("end sum " << sum << " poly size " << poly->size());
        sums.push_back(sum);
    }
    // ROS_INFO_STREAM("end");
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
    geometry_msgs::Point p;
    // normalize(plane.get_parameter_form().first);
    // normalize(plane.get_parameter_form().second);
    // ROS_ERROR_STREAM(plane.parameter_form.first << plane.parameter_form.second);
    // ROS_INFO_STREAM("x = " << d2p.x << " y = " << d2p.y);
    p = d2p.x * plane.get_parameter_form().first + d2p.y * plane.get_parameter_form().second;
    // ROS_INFO_STREAM(p << endl << endl);

    return p;
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

void Grasp_Calculator::get_grasp_point(Plane plane, geometry_msgs::Point m, double d, double alpha,
                                       geometry_msgs::PoseStamped &grasp_pose,
                                       geometry_msgs::PoseStamped &pre_grasp_pose)
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

    pre_grasp_pose.pose.orientation = grasp_pose.pose.orientation;
    normalize(r);
    pre_grasp_pose.pose.position.x = grasp_pose.pose.position.x + Gripper::GRIPPER_DEPTH * r.x;
    pre_grasp_pose.pose.position.y = grasp_pose.pose.position.y + Gripper::GRIPPER_DEPTH * r.y;
    pre_grasp_pose.pose.position.z = grasp_pose.pose.position.z + Gripper::GRIPPER_DEPTH * r.z;
}

int Grasp_Calculator::calcMeshGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
        std::vector<geometry_msgs::PoseStamped> &pre_poses,
        double gripper_depth)
{
    string gripper_group = gripper_depth == Gripper::R_GRIPPER_PALM_LENGTH ?
                           Gripper::get_r_group_name() : Gripper::get_l_group_name();


    std::vector<geometry_msgs::Point> c;
    suturo_manipulation::Mesh meshi(co);

    // 2. search for cluster with opposite normal - O(|Cluster|²)
    std::vector< std::pair<uint, uint> > opposite_cluster = meshi.get_opposite_cluster();
    int h = 0;
    for (int i = 0; i < opposite_cluster.size(); i++)
    {
        // 3. project both cluster on a plain - o(|vertices|)?
        suturo_manipulation::Plane plane = meshi.get_plane(opposite_cluster[i].first, opposite_cluster[i].second);

        std::vector<geometry_msgs::Point> p1 = meshi.create_polygon(opposite_cluster[i].first);
        std::vector<geometry_msgs::Point> p2 = meshi.create_polygon(opposite_cluster[i].second);
        if (i == 0)
        {
            // ROS_INFO_STREAM("cluser " << opposite_cluster[i].first << "  " << opposite_cluster[i].second);
            pi_->publishMarkerLine(co.id, p1, 0);
            pi_->publishMarkerLine(co.id, p2, 1000);
            // ROS_INFO_STREAM("poly 1 size: " << p1.size() << " poly2 size: " << p2.size());

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
        Path centroids = calc_poly_centroid(solution);

        DPolygon2D double_centroids;
        path_to_double_polygon(double_centroids, centroids);
        // ROS_ERROR_STREAM("!!!");
        // ROS_ERROR_STREAM("cluster 1: " << opposite_cluster[i].first << " cluster 2: " << opposite_cluster[i].second);
        // if (double_centroids.size() != 0) ROS_INFO_STREAM("centroid: " << double_centroids[0]);
        std::vector<geometry_msgs::Point> p = d2d_points_to_d3d_points(plane, double_centroids);
        for (std::vector<geometry_msgs::Point>::iterator i = p.begin(); i != p.end(); ++i)
        {
            c.push_back(*i);
        }
        // if (p.size() != 0)
        // {
        //     ROS_INFO_STREAM("centroid: " << p[0]);
        //     ROS_INFO_STREAM(" plane : " << plane.normal <<  " parameter_form: " << plane.parameter_form.first << " 2. " << plane.parameter_form.second << endl);
        // }

        //ggf 5.2 check dist to cluster
        // http://www.uninformativ.de/bin/RaytracingSchnitttests-76a577a-CC-BY.pdf

        // 6. add grasppose pointing towards centriod with differen angles - const

        for (std::vector<geometry_msgs::Point>::iterator i = p.begin(); i != p.end(); ++i)
        {
            for (double a = 0; a < 2 * M_PI; a += (M_PI / 2))
            {
                geometry_msgs::PoseStamped temp_grasp_pose;
                geometry_msgs::PoseStamped temp_pre_grasp_pose;
                temp_grasp_pose.header.frame_id = co.id;
                temp_pre_grasp_pose.header.frame_id = co.id;
                get_grasp_point(plane, *i, gripper_depth + Gripper::GRIPPER_DEPTH, a, temp_grasp_pose, temp_pre_grasp_pose);
                // h++;
                // 7. use moveit to test poses - O(much)???
                if (!pi_->check_group_object_collision(gripper_group, temp_grasp_pose, co))
                {
                    // ROS_INFO_STREAM("collision!" );
                    // ros::WallDuration(0.5).sleep();
                    poses.push_back(temp_grasp_pose);
                    pre_poses.push_back(temp_pre_grasp_pose);


                }
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
    geometry_msgs::PointStamped p = get_point_above_object(co.id);
    std::sort(poses.begin(), poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));
    std::sort(pre_poses.begin(), pre_poses.end(), boost::bind(sort_base_link_poses, _1, _2, p));
    for (int i = 0; i < poses.size() ; ++i)
    {
        pi_->publishMarker(poses[i], i);
        pi_->publishMarker(pre_poses[i], i + poses.size());
    }
    return 1;
}
