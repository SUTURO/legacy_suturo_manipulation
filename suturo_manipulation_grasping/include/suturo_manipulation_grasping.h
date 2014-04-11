#ifndef SUTURO_MANIPULATION_GRASPING
#define SUTURO_MANIPULATION_GRASPING

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <pr2_controllers_msgs/PointHeadActionResult.h>
#include <control_msgs/PointHeadActionGoal.h>

#include <visualization_msgs/Marker.h>

#include <algorithm>


class Grasping
{
protected:

    const static std::string LEFT_ARM;
    const static std::string RIGHT_ARM;

    const static double cylinder_safty_dist = 0.08;//m


    move_group_interface::MoveGroup *group_r_arm_;
    move_group_interface::MoveGroup *group_l_arm_;
    Gripper *gripper_;
    Suturo_Manipulation_Planning_Scene_Interface *pi_;
    ros::Publisher *head_publisher_;
    tf::TransformListener listener_;

    /**
     * Updated a Collisionobject with box shape depending on the gripper position.
     * Uses graspable_sides and pos_id to identify which side has been grasped.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int updateGraspedBoxPose(moveit_msgs::CollisionObject &co, int graspable_sides, int pos_id, double gripper_pose);

    /**
     * Updated a Collisionobject with cylinder shape depending on the gripper position.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int updateGraspedCylinderPose(moveit_msgs::CollisionObject &co, geometry_msgs::PoseStamped gripperPose, double gripper_pose);

    /**
     * Calculates grasp und pregrasp position for a box.
     *
     * @return >1, if succesfull
     *                  @return % 2 == 0, if the object can be grasped on the x side
     *                  @return % 3 == 0, if the object can be grasped on the y side
     *                  @return % 5 == 0, if the object can be grasped on the z side
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
     * Picks an object.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int pick(moveit_msgs::CollisionObject co, std::string arm,
             std::vector<geometry_msgs::PoseStamped> &poses, std::vector<geometry_msgs::PoseStamped> &pre_poses,
             double force);

    /**
     * Calculates grasp und pregrasp position for a box or cylinder.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int calcGraspPosition(moveit_msgs::CollisionObject co, std::vector<geometry_msgs::PoseStamped> &poses,
                          std::vector<geometry_msgs::PoseStamped> &pre_poses);

    /**
     * Publishes/updates a tf frame inside the collisionobject
     */
    void publishTfFrame(moveit_msgs::CollisionObject co);

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

    int lookAt(geometry_msgs::PoseStamped pose);

    virtual int move(move_group_interface::MoveGroup *move_group, geometry_msgs::PoseStamped desired_pose);

    void sort_grasp_poses(std::vector<geometry_msgs::PoseStamped> &poses,
                          std::vector<geometry_msgs::PoseStamped> &pre_poses);

    static const bool sort_base_link_poses(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
    {
        tf::Quaternion reference_orientation = tf::createQuaternionFromRPY(0, M_PI_2, 0);

        tf::Quaternion tf_pose1(pose1.pose.orientation.x,
                                pose1.pose.orientation.y,
                                pose1.pose.orientation.z,
                                pose1.pose.orientation.w);

        tf::Quaternion tf_pose2(pose2.pose.orientation.x,
                                pose2.pose.orientation.y,
                                pose2.pose.orientation.z,
                                pose2.pose.orientation.w);

        double angle1 = reference_orientation.angle(tf_pose1);
        double angle2 = reference_orientation.angle(tf_pose2);
        ROS_INFO_STREAM("angle1" << angle1 << " angle2" << angle2);
        return angle1 < angle2;
    }

    void transform_poses(std::string frame_id, std::vector<geometry_msgs::PoseStamped> &poses);

public:

    Grasping(Suturo_Manipulation_Planning_Scene_Interface *pi, ros::Publisher *head_publisher = NULL);

    ~Grasping();

    /**
     * Picks an object.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int pick(std::string objectName, std::string arm, double force = 50.0);

    int pick_above(std::string objectName, std::string arm, double force = 50.0);

    /**
     * Drops an object.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int dropObject(std::string object_name);

    /**
     * Detaches, whatever is attached to the arm and opens the gripper.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int drop(std::string arm);
};

#endif
