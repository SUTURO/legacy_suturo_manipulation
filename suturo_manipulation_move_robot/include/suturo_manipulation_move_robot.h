#ifndef SUTURO_MANIPULATION_MOVE_ROBOT
#define SUTURO_MANIPULATION_MOVE_ROBOT

#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <suturo_manipulation_planning_scene_interface.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

#include <boost/signals2/mutex.hpp>

#include <fstream>

#include <ReflexxesAPI.h>

#include <suturo_manipulation_2d_interpolator.h>


class Suturo_Manipulation_Move_Robot
{

private:
    const static double footprint_radius = 0.5; //0.47

    //! The node handle we'll be using
    ros::NodeHandle *nh_;
    //! We will be publishing to the "/base_controller/command" topic to issue commands
    ros::Publisher cmd_vel_pub_;
    //publishes planningscenes created during collisiondetection
    ros::Publisher coll_ps_pub_;
    // Localisation subscriber
    ros::Subscriber loc_sub_;
    // Subscriber for laserscan
    ros::Subscriber collision_sub_;
    // robot position
    geometry_msgs::PoseStamped robotPose_;
    // twist message
    geometry_msgs::Twist base_cmd_;
    // tf listener for transformations
    tf::TransformListener listener_;
    // planningscene interface to get objects
    Suturo_Manipulation_Planning_Scene_Interface *pi_;
    // target pose
    geometry_msgs::PoseStamped targetPose_;
    // target pose in base_link frame
    geometry_msgs::PoseStamped targetPoseBaseLink_;
    // twist value for z roatation
    double zTwist_;
    // bool for collision, true if collision
    std::vector<double> collisions_;
    // Angle between robot quaternion and home quaternion
    int robotToHome_;
    // Angle between robot quaternion and reverse home quaternion
    int robotToHome180_;
    // y Distance between target and robot pose at the beginning
    double yVariation_;
    // Mutex for getCollisions()
    boost::signals2::mutex mtx_;
    // Data to initialize the interpolator
    struct interpolator_2d_init_params *init_params_;
    // Inputparams to interpolate
    struct interpolator_2d_params *inp_params_;

    /**
     * Sets robotPose_ with incoming data from the localization topic.
     */
    void subscriberLocalization(const geometry_msgs::PoseStamped &robotPoseFB);

    /**
     * Calculates the distance between the base_foortprint and the environment.
     * If the distance is smaller then "footprint_radius", "inCollision_" will
     * be set to true;
     *
     */
    void subscriberCbLaserScan(const sensor_msgs::LaserScan &scan);


    /**
     * Checks if the target is arrived on x-axis.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool xCoordArrived(geometry_msgs::PoseStamped targetPose_);

    /**
     * Checks if the target is arrived on y-axis.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool yCoordArrived(geometry_msgs::PoseStamped targetPose_);

    /**
     * Checks if the target orientation is arrived.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool orientationArrived(tf::Quaternion robotOrientation, tf::Quaternion *targetOrientation);

    /**
     * Waits for incoming data from localization topic.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool checkLocalization();

    /**
     * Transforms a PoseStamped Object to a new PoseStamped Object with
     * Coordinates in base_link frame.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool transformToBaseLink(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink);

    /**
     * Transforms a PoseStamped Object to a new PoseStamped Object with
     * Coordinates in map frame.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool transformToMap(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink);

    /**
    * Calculates the Z value for rotating the robot.
    *
    * @return true, if successfull
    *                   false, otherwise
    */
    bool calculateZTwist(tf::Quaternion *targetOrientation);

    /**
    * Checks if a collision is in front of the robot.
    *
    * @return true, if collsion
    *                   false, if not
    */
    bool collisionInFront(std::vector<double> collisionsList);

    /**
    * Checks if a collision is on the left side of the robot.
    *
    * @return true, if collsion
    *                   false, if not
    */
    bool collisionOnLeft(std::vector<double> collisionsList);

    /**
    * Checks if a collision is on the right side of the robot.
    *
    * @return true, if collsion
    *                   false, if not
    */
    bool collisionOnRight(std::vector<double> collisionsList);

    /**
    * Checks the current variation on the y axis between
    * the goal and the robot position and compares it with
    * yVariation_.
    *
    * @return true, if the robot is closer to the goal
    *                   false, if not
    */
    bool checkYVariation();

    /**
     * Rotates the base from the PR2 to the target orientation from
     * the targetPose.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool rotateBase();

    /**
     * Checks if the given pose is in collision with a collisionobject.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool checkFullCollision(geometry_msgs::PoseStamped robot_pose);

public:


    Suturo_Manipulation_Move_Robot(ros::NodeHandle *nh);

    ~Suturo_Manipulation_Move_Robot();

    /**
     * Moves the base from the PR2 to the target coordinates from
     * the targetPose.
     *
     * @return true, if successfull
     *                  false, otherwise
     */
    bool driveBase(geometry_msgs::PoseStamped targetPose, double range);

    /**
     * @return collision_
     */
    std::vector<double> getLaserScanCollisions();

};

#endif
