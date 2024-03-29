#include "suturo_manipulation_move_robot.h"

//! ROS node initialization

Suturo_Manipulation_Move_Robot::Suturo_Manipulation_Move_Robot(ros::NodeHandle *nodehandle)
{
    nh_ = nodehandle;

    pi_ = new Suturo_Manipulation_Planning_Scene_Interface(nh_);

    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/base_controller/command", 1);
    coll_ps_pub_ = nh_->advertise<moveit_msgs::PlanningScene>("/suturo/collision_ps", 10);
    // localisation subscriber
    loc_sub_ = nh_->subscribe("/suturo/robot_location", 50, &Suturo_Manipulation_Move_Robot::subscriberLocalization, this);
    collision_sub_ = nh_->subscribe("/base_scan", 50, &Suturo_Manipulation_Move_Robot::subscriberCbLaserScan, this);
    ros::WallDuration(1.0).sleep();
}

Suturo_Manipulation_Move_Robot::~Suturo_Manipulation_Move_Robot()
{

}

void Suturo_Manipulation_Move_Robot::subscriberLocalization(const geometry_msgs::PoseStamped &robotPoseFB)
{
    robotPose_.header = robotPoseFB.header;
    robotPose_.pose = robotPoseFB.pose;
}

void Suturo_Manipulation_Move_Robot::subscriberCbLaserScan(const sensor_msgs::LaserScan &scan)
{
    std::vector<double> collisionsListTemp;

    for (int i = 0; i < scan.ranges.size(); i++)
    {

        //cosinussatz um abstand zu mittelpunkt zu berechnen
        double alpha = scan.angle_increment * i + 0.872664626;
        double b = scan.ranges[i];
        double c = -0.275;//dist base_link to base_laser_link
        double a = sqrt((b * b) + (c * c) + (2 * b * c * cos(alpha)));
        if (a < footprint_radius)
        {
            //angle relativ to baselink
            //~ c = -c;
            double beta = acos ( (b * b - a * a - c * c) / (2 * a * c) );
            if (alpha <= M_PI)
            {
                beta = M_PI - beta;
            }
            else
            {
                beta += M_PI;
            }
            collisionsListTemp.push_back(beta);
            // ROS_INFO_STREAM("alpha " << alpha);
            // ROS_INFO_STREAM("beta " << beta);
        }
    }
    mtx_.lock();
    collisions_ = collisionsListTemp;
    mtx_.unlock();
    // if (!collisions_.empty()){
    //    ROS_ERROR_STREAM("COLLISIONelf" );
    // }
}

bool Suturo_Manipulation_Move_Robot::checkFullCollision(geometry_msgs::PoseStamped robot_pose)
{
    moveit_msgs::PlanningScene ps;
    pi_->getPlanningScene(ps);

    ps.robot_state.multi_dof_joint_state.header.frame_id = robot_pose.header.frame_id;

    ps.robot_state.multi_dof_joint_state.joint_transforms[0].translation.x = robot_pose.pose.position.x;
    ps.robot_state.multi_dof_joint_state.joint_transforms[0].translation.y = robot_pose.pose.position.y;
    ps.robot_state.multi_dof_joint_state.joint_transforms[0].translation.z = robot_pose.pose.position.z;
    ps.robot_state.multi_dof_joint_state.joint_transforms[0].rotation.x = robot_pose.pose.orientation.x;
    ps.robot_state.multi_dof_joint_state.joint_transforms[0].rotation.y = robot_pose.pose.orientation.y;
    ps.robot_state.multi_dof_joint_state.joint_transforms[0].rotation.z = robot_pose.pose.orientation.z;
    ps.robot_state.multi_dof_joint_state.joint_transforms[0].rotation.w = robot_pose.pose.orientation.w;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    planning_scene.setPlanningSceneMsg(ps);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result);

    // coll_ps_pub_.publish(ps);

    return collision_result.collision;
}

std::vector<double> Suturo_Manipulation_Move_Robot::getLaserScanCollisions()
{
    return collisions_;
}

bool Suturo_Manipulation_Move_Robot::collisionInFront(std::vector<double> collisionsList)
{

    for (int position = collisionsList.size() - 1; position >= 0; position--)
    {
        // between 90 degrees and 270 degrees
        if (collisionsList.at(position) > 1.57 && collisionsList.at(position) < 4.71)
        {
            //ROS_ERROR_STREAM("Collision in front!");
            return true;
        }
    }

    return false;
}

bool Suturo_Manipulation_Move_Robot::collisionOnRight(std::vector<double> collisionsList)
{

    for (int position = collisionsList.size() - 1; position >= 0; position--)
    {
        // wenn auf der rechten seite des robos 90 grad ist
        // between 45 degrees and 135 degrees
        if (collisionsList.at(position) > 0.79 && collisionsList.at(position) < 2.36)
        {
            //ROS_ERROR_STREAM("Collision on right side!");
            return true;
        }
    }

    return false;
}

bool Suturo_Manipulation_Move_Robot::collisionOnLeft(std::vector<double> collisionsList)
{

    for (int position = collisionsList.size() - 1; position >= 0; position--)
    {
        // wenn auf der linken seite des robos 270 grad ist
        // between 225 degrees and 315 degrees
        if (collisionsList.at(position) > 3.93 && collisionsList.at(position) < 5.5)
        {
            //  ROS_ERROR_STREAM("Collision on left side!");
            return true;
        }
    }
    return false;
}

bool Suturo_Manipulation_Move_Robot::checkYVariation()
{
    double currentVariation = abs(targetPose_.pose.position.y - robotPose_.pose.position.y);

    return currentVariation <= (yVariation_ + 0.05);
}


bool Suturo_Manipulation_Move_Robot::checkLocalization()
{
    return (robotPose_.pose.position.x != 0 || robotPose_.pose.position.y != 0 || robotPose_.pose.position.z != 0);
}

bool Suturo_Manipulation_Move_Robot::xCoordArrived(geometry_msgs::PoseStamped targetPose)
{
    return (robotPose_.pose.position.x < targetPose.pose.position.x + 0.02 && robotPose_.pose.position.x > targetPose.pose.position.x - 0.02);
}

bool Suturo_Manipulation_Move_Robot::yCoordArrived(geometry_msgs::PoseStamped targetPose)
{
    return (robotPose_.pose.position.y < targetPose.pose.position.y + 0.02 && robotPose_.pose.position.y > targetPose.pose.position.y - 0.02);
}

bool Suturo_Manipulation_Move_Robot::orientationArrived(tf::Quaternion robotOrientation, tf::Quaternion *targetOrientation)
{
    return (targetOrientation->angle(robotOrientation) < 0.05) && (targetOrientation->angle(robotOrientation) > -0.05);
}

bool Suturo_Manipulation_Move_Robot::transformToBaseLink(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink)
{
    try
    {
        //transform pose to base_link
        listener_.transformPose("/base_link", pose, poseInBaseLink);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
        return false;
    }
    return true;
}

bool Suturo_Manipulation_Move_Robot::transformToMap(geometry_msgs::PoseStamped pose, geometry_msgs::PoseStamped &poseInBaseLink)
{
    try
    {
        //transform pose to base_link
        listener_.transformPose("/map", pose, poseInBaseLink);
    }
    catch (...)
    {
        ROS_ERROR_STREAM("ERROR: Transformation failed.");
        return false;
    }
    return true;
}

bool Suturo_Manipulation_Move_Robot::calculateZTwist(tf::Quaternion *targetQuaternion)
{
    // TODO: fixen, fährt derzeit immer in die entgegengesetzte Richtung
    zTwist_ = 0;

    geometry_msgs::PoseStamped homePose;
    geometry_msgs::PoseStamped homePose180;

    geometry_msgs::PoseStamped homePoseBase;
    geometry_msgs::PoseStamped homePose180Base;

    homePose.header.frame_id = "/map";
    homePose180.header.frame_id = "/map";

    homePose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
    homePose180.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    transformToBaseLink(homePose, homePoseBase);
    transformToBaseLink(homePose180, homePose180Base);

    tf::Quaternion robotPoseQuaternion(0, 0, 0, 1);
    tf::Quaternion homePoseOuaternion(homePoseBase.pose.orientation.x, homePoseBase.pose.orientation.y, homePoseBase.pose.orientation.z, homePoseBase.pose.orientation.w);
    tf::Quaternion homePose180Quaternion(homePose180Base.pose.orientation.x, homePose180Base.pose.orientation.y, homePose180Base.pose.orientation.z, homePose180Base.pose.orientation.w);

    robotToHome_ = robotPoseQuaternion.angle(homePoseOuaternion);
    robotToHome180_ = robotPoseQuaternion.angle(homePose180Quaternion);

    if ( robotToHome_ < robotToHome180_)
    {
        zTwist_ = 0.2;
        return true;
    }
    else
    {
        zTwist_ = -0.2;
        return true;
    }
    ROS_WARN("Calculation of z twist failed, rotating and moving aborted!");
    return false;
}


bool Suturo_Manipulation_Move_Robot::rotateBase()
{

    tf::Quaternion *targetQuaternion = new tf::Quaternion(targetPoseBaseLink_.pose.orientation.x, targetPoseBaseLink_.pose.orientation.y, targetPoseBaseLink_.pose.orientation.z, targetPoseBaseLink_.pose.orientation.w);
    tf::Quaternion robotOrientation(0, 0, 0, 1);

    std::vector<double> collisionsListRotation;

    ROS_INFO("Begin to rotate base");

    if (calculateZTwist(targetQuaternion))
    {
        while (nh_->ok() && !orientationArrived(robotOrientation, targetQuaternion) && transformToBaseLink(targetPose_, targetPoseBaseLink_))
        {

            mtx_.lock();
            collisionsListRotation = getLaserScanCollisions();
            mtx_.unlock();

            if (collisionInFront(collisionsListRotation))
            {
                base_cmd_.linear.x = (-0.1);

                cmd_vel_pub_.publish(base_cmd_);
                base_cmd_.linear.x = 0;
                continue;
            }

            base_cmd_.angular.z = zTwist_;
            cmd_vel_pub_.publish(base_cmd_);

            targetQuaternion = new tf::Quaternion(targetPoseBaseLink_.pose.orientation.x, targetPoseBaseLink_.pose.orientation.y, targetPoseBaseLink_.pose.orientation.z, targetPoseBaseLink_.pose.orientation.w);
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool Suturo_Manipulation_Move_Robot::driveBase(geometry_msgs::PoseStamped targetPose, double range)
{

    // reset values
    base_cmd_.linear.x = 0;
    base_cmd_.linear.y = 0;
    base_cmd_.angular.z = 0;
    struct interpolator_2d_result interpoolator_result_;

    targetPose_ = targetPose;

    //    if (checkCollision(targetPose_))
    //    {
    //        ROS_ERROR_STREAM("targetpose in collision!");
    //        return false;
    //    }

    while (!checkLocalization())
    {
        // Wait for localization...
    }

    if (!transformToBaseLink(targetPose_, targetPoseBaseLink_))
    {
        ROS_ERROR_STREAM("Transformation to base_link failed! Moving aborted!");
        return false;
    }

    if (rotateBase())
    {
        ROS_INFO("rotateBase done");
    }
    else
    {
        return false;
    }

    ROS_INFO("begin to move");

    double yTwist;
    bool moveLeft = false;
    bool moveRight = false;
    std::vector<double> collisionsList;

    mtx_.lock();
    collisionsList = getLaserScanCollisions();
    mtx_.unlock();

    yVariation_ = abs(targetPose_.pose.position.y - robotPose_.pose.position.y);

    Suturo_Manipulation_2d_Interpolator *interpolator = new Suturo_Manipulation_2d_Interpolator();

    init_params_ = new interpolator_2d_init_params();
    // cycle time auf hertzrate des topics anpassen
    init_params_->cycle_time_ =  0.1;
    init_params_->vel_limit_ =  0.2;
    init_params_->acc_limit_ =  0.2;
    init_params_->jerk_limit_ =  0.2;
    init_params_->range_ = range;

    interpolator->init(*init_params_);

    inp_params_ = new interpolator_2d_params();

    inp_params_->target_pose_.x_ = targetPoseBaseLink_.pose.position.x;
    inp_params_->target_pose_.y_ = targetPoseBaseLink_.pose.position.y;
    inp_params_->target_pose_.reference_ = targetPoseBaseLink_.header.frame_id;

    inp_params_->robot_pose_.x_ = 0;
    inp_params_->robot_pose_.y_ = 0;
    inp_params_->robot_pose_.reference_ = "/base_link";

    while (nh_->ok() && interpoolator_result_.result_value_ != ReflexxesAPI::RML_FINAL_STATE_REACHED)
    {
        mtx_.lock();
        collisionsList = getLaserScanCollisions();
        mtx_.unlock();

        // reset all the values!!!!11elf
        base_cmd_.linear.x = 0;
        base_cmd_.linear.y = 0;
        base_cmd_.angular.z = 0;

        interpoolator_result_ = interpolator->interpolate(*inp_params_);

        transformToBaseLink(targetPose_, targetPoseBaseLink_);

        // TODO: Kollision vernünftig behandeln: Was ist wenn auf y das Ziel erreicht, aber auf x ne Kollision? Oder wenn auf x und y ne Kollision? Muss behandelt werden
        // Idee 1: Bei erster Kollision das jeweilige Ziel der Achse auf die Robopose setzen
        // => Problem: Wenn Kollision nur kurzfristig, kann Ziel nicht mehr angefahren werden
        // Idee 2: Solange in Kollision gehen, bis auf der nicht kollidierenden Achse das Ziel erreicht ist, dann das Kollidierende Ziel auf die robotPose setzen
        // => Problem: Was ist, wenn x und y kollidieren?
        // Idee 3: Mit xInCollision und yInCollision als bools arbeiten, wenn eine in Kollision, die andere zum Ziel bringen, wenn einer am Ziel, der andere in Kollision => Ziel erreicht
        // Wenn beide auf Kollision => Ziel erreicht
        // Problem allgemein: Wie dem Planning mitteilen, dass Probleme aufgetreten sind?
        if (interpoolator_result_.twist_.xdot_ != 0)
        {
            if (inp_params_->robot_pose_.x_ > targetPoseBaseLink_.pose.position.x)
            {
                base_cmd_.linear.x = 0;
                inp_params_->target_pose_.x_ = inp_params_->robot_pose_.x_;
                ROS_WARN("Can't move back!");
            }
            else if (collisionInFront(collisionsList))
            {
                base_cmd_.linear.x = 0;
                inp_params_->target_pose_.x_ = inp_params_->robot_pose_.x_;
                ROS_WARN("Collision in Front, can't move forward!");
            }
            else
            {
                base_cmd_.linear.x = interpoolator_result_.twist_.xdot_;
                inp_params_->target_pose_.x_ = targetPoseBaseLink_.pose.position.x;
            }
        }

        // wenn y > 0 links vom robo ist und y < 0 rechts vom robo
        if ((interpoolator_result_.twist_.ydot_ > 0 && collisionOnLeft(collisionsList)) || (interpoolator_result_.twist_.ydot_ < 0 && collisionOnRight(collisionsList)))
        {
            base_cmd_.linear.y = 0;
            inp_params_->target_pose_.y_ = inp_params_->robot_pose_.y_;
            ROS_WARN("Collision on side, can't move!");
        }
        else
        {
            base_cmd_.linear.y = interpoolator_result_.twist_.ydot_;
            inp_params_->target_pose_.y_ = targetPoseBaseLink_.pose.position.y;
        }

        // Create a PoseStamped object with data from interpolator and transform to map
        // geometry_msgs::PoseStamped interpolatedPose;
        // interpolatedPose = targetPoseBaseLink_;
        // interpolatedPose.pose.position.x = interpoolator_result_.int_pose_.x_;
        // interpolatedPose.pose.position.x = interpoolator_result_.int_pose_.y_;
        // transformToMap(interpolatedPose, interpolatedPose);
        // interpolatedPose.pose.orientation = robotPose_.pose.orientation;

        // // Check for collisions in planningscene with moveit
        // if (checkFullCollision(interpolatedPose))
        // {
        //     ROS_WARN("Collision in PlanningScene");
        //     inp_params_->target_pose_.x_ = inp_params_->robot_pose_.x_;
        //     inp_params_->target_pose_.y_ = inp_params_->robot_pose_.y_;
        // }

        inp_params_->target_pose_.reference_ = targetPoseBaseLink_.header.frame_id;
        inp_params_->twist_=interpoolator_result_.twist_;

        cmd_vel_pub_.publish(base_cmd_);
    }

    ROS_INFO_STREAM(targetPose_);
    ROS_INFO_STREAM(robotPose_);

    // Finish him!
    return true;
}
