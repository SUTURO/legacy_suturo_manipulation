#ifndef SUTURO_MANIPULATION_GRASP_CALCULATOR
#define SUTURO_MANIPULATION_GRASP_CALCULATOR

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>

// #include <suturo_manipulation_mesh_loader.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>
#include <control_msgs/PointHeadActionGoal.h>

#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <clipper.h>

#include <suturo_manipulation_mesh.h>

// #include <ostream>

// using namespace suturo_manipulation;

class Grasp_Calculator
{
protected:

    const static std::string LEFT_ARM;
    const static std::string RIGHT_ARM;

    const static double cylinder_safty_dist = 0.08;//m

    Gripper *gripper_;
    Suturo_Manipulation_Planning_Scene_Interface *pi_;
    ros::Publisher *head_publisher_;
    tf::TransformListener listener_;

    /**
     * Calculates grasp und pregrasp position for a box.
     *
     * @return >1, if succesfull
     *                  0, otherwise
     */
    int calcBoxGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                             std::vector<geometry_msgs::PoseStamped> &pre_poses,
                             double gripper_depth);

    /**
     * Calculates grasp und pregrasp position for a cylinder.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                  std::vector<geometry_msgs::PoseStamped> &pre_poses,
                                  double gripper_depth);

    /**
     * Adds Grasppositions possible from above and below, to the vactors.
     */
    void addGraspPositionsZ(double d, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses,
                            std::vector<geometry_msgs::PoseStamped> &pre_poses,
                            double gripper_depth);

    /**
     * Adds Grasppositions possible from the front and behind, to the vactors.
     */
    void addGraspPositionsX(double h, double d, double rotation, std::string frame_id,
                            std::vector<geometry_msgs::PoseStamped> &poses,
                            std::vector<geometry_msgs::PoseStamped> &pre_poses,
                            double gripper_depth);

    /**
     * Adds Grasppositions possible from left and right, to the vactors.
     */
    void addGraspPositionsY(double h, double d, double rotation, std::string frame_id,
                            std::vector<geometry_msgs::PoseStamped> &poses,
                            std::vector<geometry_msgs::PoseStamped> &pre_poses,
                            double gripper_depth);

public:

    struct DoublePoint2D
    {
        double x;
        double y;
        bool operator==(const DoublePoint2D &d2p) const
        {
            return d2p.x == this->x && d2p.y == this->y;
        }
        bool operator!=(const DoublePoint2D &d2p) const
        {
            return d2p.x != this->x || d2p.y != this->y;
        }

    };


    typedef std::vector<DoublePoint2D> DPolygon2D;

    Grasp_Calculator(Suturo_Manipulation_Planning_Scene_Interface *pi);

    ~Grasp_Calculator();

    /**
     * Calculates grasp und pregrasp position for a box or cylinder.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                          std::vector<geometry_msgs::PoseStamped> &pre_poses,
                          double gripper_depth);

    /**
     * Calculates the scalarproduct of two vectors represented by Points
     *
     * @return scalarproduct
     */
    static const double scalarproduct(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return (p1.x * p2.x) +
               (p1.y * p2.y) +
               (p1.z * p2.z);
    }

    /**
     * Calculates the angle of two vectors represented by Points
     *
     * @return angle
     */
    static const double get_angle(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return acos (Grasp_Calculator::scalarproduct(p1, p2) /
                     (sqrt(Grasp_Calculator::scalarproduct(p1, p1)) * sqrt(Grasp_Calculator::scalarproduct(p2, p2))));
    }

    /**
     * Calculates a point that is above an object
     *
     * @return false, if tf fails
     */
    bool get_point_above_object(std::string object_id, geometry_msgs::PointStamped &p);

    /**
     * Calculates graspposition for a mesh.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int calcMeshGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                              std::vector<geometry_msgs::PoseStamped> &pre_poses,
                              double gripper_depth);

    /**
     * Projects a list of Points onto an plane.
     *
     * @return 2 dimensional polygon
     */
    DPolygon2D project_polygon_to_plane(suturo_manipulation::Plane plane, std::vector<geometry_msgs::Point> polygon);

    /**
     * Transforms a double polygon to a int polyon, by multipling everything with a very high number.
     *
     */
    void double_polygon_to_path(DPolygon2D double_polygon, ClipperLib::Paths &int_polygon);

    /**
     * Transforms a int polygon to a double polyon, by deviding everything with a very high number.
     *
     */
    void path_to_double_polygon(DPolygon2D &double_polygon, ClipperLib::Path int_polygon);

    /**
     * Calculates the centroids of a polygon list
     *
     * @return list of centroids
     */
    std::vector<ClipperLib::IntPoint> calc_poly_centroid(ClipperLib::Paths polygon);

    /**
     * Transforms a list of 2D Points on a Plane into 3D coordinates.
     *
     * @return list of 3d points
     */
    std::vector<geometry_msgs::Point> d2d_points_to_d3d_points(suturo_manipulation::Plane plane, DPolygon2D polygon);

    /**
     * Transforms a 2D Point on a Plane into 3D coordinates.
     *
     * @return 3d point
     */
    geometry_msgs::Point d2d_point_to_d3d_point(suturo_manipulation::Plane plane, DoublePoint2D d2p);

    /**
     * @return a Quaternon pointign from "from" to "to", with a roll defined by "roll"
     *
     */
    geometry_msgs::Quaternion get_quaternion_from_points(geometry_msgs::Point from,
            geometry_msgs::Point to,
            geometry_msgs::Point roll);

    /**
     * Calculates the cross product of two points.
     *
     * @return the cross product
     */
    geometry_msgs::Point cross_product(geometry_msgs::Point p1, geometry_msgs::Point p2);

    /**
     * Calculates a grasppoint above m. The point is in the plane at an angle of alpha around m.
     *
     * @return false, if it fails
     */
    bool get_grasp_point(suturo_manipulation::Plane plane, geometry_msgs::Point m, double d, double alpha,
                         geometry_msgs::PoseStamped &grasp_pose, geometry_msgs::PoseStamped &pre_grasp_pose,
                         suturo_manipulation::Mesh meshi);
};


#endif
