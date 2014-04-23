#include "suturo_manipulation_grasp_calculator.h"

using namespace std;

Grasp_Calculator::Grasp_Calculator(Suturo_Manipulation_Planning_Scene_Interface *pi)
{
    //initialize private variables
    // group_r_arm_ = new move_group_interface::MoveGroup(RIGHT_ARM);
    // group_r_arm_->setPlanningTime(5.0);

    // group_l_arm_ = new move_group_interface::MoveGroup(LEFT_ARM);
    // group_l_arm_->setPlanningTime(5.0);

    // head_publisher_ = head_publisher;
    // gripper_ = new Gripper();

    //pi nicht selbst erstellen, weil das Weiterreichen des nodehandle über 2 Klassen rumbugt :(
    pi_ = pi;

    // //wait because ros
    // ros::WallDuration(0.5).sleep();
}

Grasp_Calculator::~Grasp_Calculator()
{

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

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = gripper_depth + d;
    pre_pose = pose;
    pre_pose.pose.position.z += Gripper::GRIPPER_DEPTH - 0.05;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);


    //grasp from below
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI_2, rotation);

    pose.pose.position.z = 0 - gripper_depth - d;
    pre_pose = pose;
    pre_pose.pose.position.z -= Gripper::GRIPPER_DEPTH - 0.05;

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

    //grasp from the front
    pose.pose.position.x = 0 - gripper_depth - d - 0.005;
    pose.pose.position.y = 0;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, 0);

    pre_pose = pose;
    pre_pose.pose.position.x -= Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from behind
    pose.pose.position.x = gripper_depth + d + 0.005;
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

    //grasp from the left
    pose.pose.position.x = 0;
    pose.pose.position.y = gripper_depth + d + 0.005;
    pose.pose.position.z = h;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rotation, 0, -M_PI_2);

    pre_pose = pose;
    pre_pose.pose.position.y += Gripper::GRIPPER_DEPTH;

    poses.push_back(pose);
    pre_poses.push_back(pre_pose);

    //grasp from right
    pose.pose.position.y = 0 - gripper_depth - d - 0.005;
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

void Grasp_Calculator::build_cluster(shapes::Mesh *mesh, std::vector<Cluster> &clusters)
{
    double threshold = 0.25;

    //create better useable datatypes
    std::vector<geometry_msgs::Point> mesh_vertices;
    for (int i = 0; i < mesh->vertex_count; i++)
    {
        geometry_msgs::Point p;
        p.x = mesh->vertices[3 * i];
        p.y = mesh->vertices[(3 * i) + 1];
        p.z = mesh->vertices[(3 * i) + 2];
        mesh_vertices.push_back(p);
    }

    std::vector<Triangle> mesh_triangles;
    mesh->computeTriangleNormals();
    for (int i = 0; i < mesh->triangle_count; i++)
    {
        Triangle t;
        t.triangle.push_back(mesh->triangles[(i * 3)]);
        t.triangle.push_back(mesh->triangles[(i * 3) + 1]);
        t.triangle.push_back(mesh->triangles[(i * 3) + 2]);
        //sort for later intersection
        sort(t.triangle.begin(), t.triangle.end());
        t.normal.x = mesh->triangle_normals[i * 3];
        t.normal.y = mesh->triangle_normals[(i * 3) + 1];
        t.normal.z = mesh->triangle_normals[(i * 3) + 2];
        mesh_triangles.push_back(t);
    }
    while (mesh_triangles.size() > 0)
    {
        // ROS_INFO_STREAM(mesh_triangles.size());
        Cluster c;
        c.triangles.push_back(mesh_triangles[0]);
        mesh_triangles.erase(mesh_triangles.begin());
        c.vertices = &mesh_vertices;
        c.normal = c.triangles[0].normal;
        for (int i = 0; i < mesh_triangles.size();)
        {
            //has the triangle a similar normal?
            double a = get_angle(c.normal, mesh_triangles[i].normal);
            if (a < threshold)
            {
                //check for two connections
                std::vector<unsigned int> v3;
                for (int j = 0; j < c.triangles.size(); j++)
                {
                    std::set_intersection(c.triangles[j].triangle.begin(), c.triangles[j].triangle.end(), mesh_triangles[i].triangle.begin(),
                                          mesh_triangles[i].triangle.end(), back_inserter(v3));
                    if (v3.size() >= 2)
                        break;
                }
                //is the triangle connected to another triangle of the cluster?
                if (v3.size() >= 2)
                {
                    c.triangles.push_back(mesh_triangles[i]);
                    mesh_triangles.erase(mesh_triangles.begin() + i);
                    //start over, because new triangles could be connected
                    i = 0;
                }
                else i++;
            }
            else i++;
        }
        clusters.push_back(c);
    }
}

void Grasp_Calculator::search_for_opposte_cluster(std::vector<Cluster> clusters, std::vector< std::pair<Cluster, Cluster> > &opposite_cluster)
{
    double threshold = 0.2;
    for (int i = 0; i < clusters.size(); i++)
    {
        for (int j = i + 1; j < clusters.size(); j++)
        {
            if (M_PI - get_angle(clusters[i].normal, clusters[j].normal) < threshold)
            {
                std::pair<Cluster, Cluster> c_pair(clusters[i], clusters[j]);
                opposite_cluster.push_back(c_pair);
            }
        }
    }
}

Grasp_Calculator::Plane Grasp_Calculator::create_plane(geometry_msgs::Point normal, geometry_msgs::Point normal2)
{
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

    Grasp_Calculator::Plane_parameter pp(p1, p2);
    Grasp_Calculator::Plane plane;
    plane.pp = pp;
    plane.normal = plane_normal;

    return plane;
}

void Grasp_Calculator::create_polygon(Cluster c, Double_path, Muh muh)
{

}

void Grasp_Calculator::calcMeshGraspPosition(shapes::Mesh *mesh)
{
    // 1. build simple polygon cluster - O(|Triangle|²)?
    // 1.1. take random triangle
    // 1.2. search for triangle with similar normal and with >=2 contacts
    // (1.2.1 update representative triangle)
    // 1.3. repeat until no new trianlge added
    // (1.4 remove small clusters)
    // std::vector<Cluster> clusters;
    // Muh muh;
    // build_cluster(mesh, clusters, muh);

    // // 2. search for cluster with opposite normal - O(|Cluster|²)
    // std::vector< std::pair<uint, uint> > opposite_cluster;
    // search_for_opposte_cluster(clusters, opposite_cluster);

    // for (int i = 0; i < opposite_cluster.size(); i++)
    // {
    //     // 3. project both cluster on a plain - o(|vertices|)?
    //     Grasp_Calculator::Plane plane = create_plane(opposite_cluster[i].first, opposite_cluster[i].second, clusters);
        
    //     std::vector<uint> p1;
    //     std::vector<uint> p2;
    //     create_polygon(opposite_cluster[i].first, p1, muh, clusters);
    //     create_polygon(opposite_cluster[i].second, p2, muh, clusters);

    //     project_polygon_to_plane(plane, p1);
    //     project_polygon_to_plane(plane, p2);

    //     ClipperLib::Paths polygon1(1);
    //     ClipperLib::Paths polygon2(1);
    //     double_polygon_to_path(p1, polygon1);
    //     double_polygon_to_path(p2, polygon2);


    //     // // 4. calc polygon intersection - O(|vertices|²)?
    //     //     http://www.integis.ch/documents/ISem_Opprecht_Overlay_2002-02-28.pdf
    //     //     http://www.cs.man.ac.uk/~toby/alan/software/gpc.html
    //     ClipperLib::Clipper clpr;
    //     clpr.AddPaths(polygon1, ClipperLib::ptSubject, true);
    //     clpr.AddPaths(polygon2, ClipperLib::ptClip, true);
    //     ClipperLib::Paths solution;
    //     clpr.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);


    //     // 5. calc centriod - O(|vertices|)
    //     // 6. add grasppose pointing towards centriod with differen angles - const
    //     // 7. use moveit to test poses - O(much)???
    // }


}
