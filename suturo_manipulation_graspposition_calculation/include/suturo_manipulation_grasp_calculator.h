#ifndef SUTURO_MANIPULATION_GRASP_CALCULATOR
#define SUTURO_MANIPULATION_GRASP_CALCULATOR

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>
#include <control_msgs/PointHeadActionGoal.h>

#include <visualization_msgs/Marker.h>

#include <algorithm>


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
                             std::vector<geometry_msgs::PoseStamped> &pre_poses);

    /**
     * Calculates grasp und pregrasp position for a cylinder.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int calcCylinderGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                                  std::vector<geometry_msgs::PoseStamped> &pre_poses);

    /**
     * Adds Grasppositions possible from above and below, to the vactors.
     */
    void addGraspPositionsZ(double d, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses,
                            std::vector<geometry_msgs::PoseStamped> &pre_poses);

    /**
     * Adds Grasppositions possible from the front and behind, to the vactors.
     */
    void addGraspPositionsX(double h, double d, double rotation, std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses,
                            std::vector<geometry_msgs::PoseStamped> &pre_poses);

    /**
     * Adds Grasppositions possible from left and right, to the vactors.
     */
    void addGraspPositionsY(double h, double d, double rotation, std::string frame_id,
                            std::vector<geometry_msgs::PoseStamped> &poses,
                            std::vector<geometry_msgs::PoseStamped> &pre_poses);

    static const bool sort_base_link_poses(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
    {
        geometry_msgs::Quaternion reference_q = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, 0);
        tf::Quaternion reference_orientation(reference_q.x,
                                             reference_q.y,
                                             reference_q.z,
                                             reference_q.w);
        // ROS_INFO_STREAM("reference " << reference_orientation.getAxis().getX() << " " <<
        //     reference_orientation.getAxis().getY() << " " <<
        //     reference_orientation.getAxis().getZ());

        tf::Quaternion tf_pose1(pose1.pose.orientation.x,
                                pose1.pose.orientation.y,
                                pose1.pose.orientation.z,
                                pose1.pose.orientation.w);
        // ROS_INFO_STREAM("pose1 " << tf_pose1.getAxis().getX() << " " <<
        //     tf_pose1.getAxis().getY() << "" <<
        //     tf_pose1.getAxis().getZ());
        tf::Quaternion tf_pose2(pose2.pose.orientation.x,
                                pose2.pose.orientation.y,
                                pose2.pose.orientation.z,
                                pose2.pose.orientation.w);
        // ROS_INFO_STREAM("pose2 " << tf_pose2.getAxis().getX() << " " <<
        //     tf_pose2.getAxis().getY() << "" <<
        //     tf_pose2.getAxis().getZ());
        double angle1 = reference_orientation.angle(tf_pose1);
        double angle2 = reference_orientation.angle(tf_pose2);
        // ROS_INFO_STREAM("angle1 " << angle1 << " angle2 " << angle2);
        return angle1 < angle2;
    }

    void transform_poses(std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses);

public:

    Grasp_Calculator(Suturo_Manipulation_Planning_Scene_Interface *pi);

    ~Grasp_Calculator();

    /**
     * Calculates grasp und pregrasp position for a box or cylinder.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                          std::vector<geometry_msgs::PoseStamped> &pre_poses);

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
};

#endif
