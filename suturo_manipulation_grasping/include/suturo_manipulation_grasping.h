#ifndef SUTURO_MANIPULATION_GRASPING
#define SUTURO_MANIPULATION_GRASPING

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>

#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_grasp_calculator.h>

#include <pr2_controllers_msgs/PointHeadActionResult.h>
#include <control_msgs/PointHeadActionGoal.h>


#include <algorithm>


class Grasping
{
protected:

    const static std::string LEFT_ARM;
    const static std::string RIGHT_ARM;

    move_group_interface::MoveGroup *group_r_arm_;
    move_group_interface::MoveGroup *group_l_arm_;
    Gripper *r_gripper_;
    Gripper *l_gripper_;
    Grasp_Calculator *grasp_calculator_;
    Suturo_Manipulation_Planning_Scene_Interface *pi_;
    ros::Publisher *head_publisher_;
    tf::TransformListener listener_;
    ros::NodeHandle* nh_;

    /**
     * Publishes/updates a tf frame inside the collisionobject
     */
    void publishTfFrame(moveit_msgs::CollisionObject co);

    int lookAt(geometry_msgs::PoseStamped pose);

    virtual int move(move_group_interface::MoveGroup *move_group, 
        geometry_msgs::PoseStamped desired_pose,
        moveit_msgs::CollisionObject co);

    int get_attached_object(std::string arm, std::string object_name, moveit_msgs::CollisionObject &co);

    bool get_move_group(std::string arm, move_group_interface::MoveGroup *&move_group);
    bool get_gripper(std::string arm, Gripper *&gripper_);



public:

    Grasping(ros::NodeHandle* nh, Suturo_Manipulation_Planning_Scene_Interface* pi, ros::Publisher* head_publisher=NULL);

    ~Grasping();

    /**
     * Picks an object.
     *
     * @return 1, if succesfull
     *                  0, otherwise
     */
    int pick(std::string objectName, std::string arm, double force = 50.0);

    int pick_above(std::string objectName, std::string arm, double tolerance, double force = 50.0);

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
