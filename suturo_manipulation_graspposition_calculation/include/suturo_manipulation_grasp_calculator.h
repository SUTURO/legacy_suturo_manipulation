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

    void transform_poses(std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses);

public:

    struct DoublePoint2D
    {
        double x;
        double y;
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

    static const double scalarproduct(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return (p1.x * p2.x) +
               (p1.y * p2.y) +
               (p1.z * p2.z);
    }

    static const double get_angle(geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return acos (Grasp_Calculator::scalarproduct(p1, p2) /
                     (sqrt(Grasp_Calculator::scalarproduct(p1, p1)) * sqrt(Grasp_Calculator::scalarproduct(p2, p2))));
    }

    geometry_msgs::PointStamped get_point_above_object(std::string object_id);

    void calcMeshGraspPosition(shapes::Mesh *mesh);

    DPolygon2D project_polygon_to_plane(suturo_manipulation::Plane plane, std::vector<geometry_msgs::Point> polygon);

    void double_polygon_to_path(DPolygon2D double_polygon, ClipperLib::Paths &int_polygon);

    geometry_msgs::Point get_point_of_intersection(suturo_manipulation::Plane plane, geometry_msgs::Point p);

};


#endif
