#include "suturo_manipulation_move_robot.h"

using namespace std;

//! ROS node initialization
Suturo_Manipulation_Move_Robot::Suturo_Manipulation_Move_Robot(ros::NodeHandle *nodehandle)
{
    nh_ = nodehandle;

    pi_ = new Suturo_Manipulation_Planning_Scene_Interface(nh_);

    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/base_controller/command", 1);
    // localisation subscriber
    loc_sub_ = nh_->subscribe("/suturo/robot_location", 50, &Suturo_Manipulation_Move_Robot::subscriberCb, this);
    collision_sub_ = nh_->subscribe("/base_scan", 50, &Suturo_Manipulation_Move_Robot::subscriberCbLaserScan, this);
    ros::WallDuration(1.0).sleep();
}

Suturo_Manipulation_Move_Robot::~Suturo_Manipulation_Move_Robot()
{

}

void Suturo_Manipulation_Move_Robot::subscriberCb(const geometry_msgs::PoseStamped &robotPoseFB)
{
    robotPose_.header = robotPoseFB.header;
    robotPose_.pose = robotPoseFB.pose;
}

bool Suturo_Manipulation_Move_Robot::checkFullCollision(double danger_zone)
{
    moveit_msgs::PlanningScene ps;
    pi_->getPlanningScene(ps);

    //increase collisionobject size
    for (int i = 0; i < ps.world.collision_objects.size(); i++)
    {
        moveit_msgs::CollisionObject &co = ps.world.collision_objects[i];
        if (co.primitives[0].type != shape_msgs::SolidPrimitive::BOX)
        {
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] += danger_zone;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] += danger_zone;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] += danger_zone;
        }
        else if (co.primitives[0].type != shape_msgs::SolidPrimitive::CYLINDER)
        {
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] += danger_zone;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] += danger_zone;
        }
    }

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    planning_scene.setPlanningSceneMsg(ps);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result);


    return collision_result.collision;
}

bool Suturo_Manipulation_Move_Robot::checkCollision(geometry_msgs::PoseStamped targetPose)
{
    //get all collisionobjects
    std::vector<moveit_msgs::CollisionObject> cos;

    if (!pi_->getObjects(cos)) return true;
    for (std::vector<moveit_msgs::CollisionObject>::iterator co = cos.begin(); co != cos.end(); ++co)
    {
        if (co->primitive_poses[0].orientation.x == 0 &&
                co->primitive_poses[0].orientation.y == 0 &&
                co->primitive_poses[0].orientation.z == 0 &&
                co->primitive_poses[0].orientation.w == 1 )
        {
            //andere orientierung ist zu schwer ;(
            //aber alle gegenstände von knowledge werden mit dieser orientierung gepublisht
            //todo? Höhe das Objekte beachten?

            //transform goalpose into objectframe
            listener_.transformPose(co->header.frame_id, targetPose, targetPose);

            //check dist
            if (co->primitives[0].type == shape_msgs::SolidPrimitive::BOX)
            {
                double x_size = co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
                double y_size = co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
                double d_x = abs(co->primitive_poses[0].position.x - targetPose.pose.position.x);
                double d_y = abs(co->primitive_poses[0].position.y - targetPose.pose.position.y);
                if (d_x <= footprint_radius + x_size / 2 && d_y <= footprint_radius + y_size / 2) return true;
            }
        }
    }

    return false;
}

bool Suturo_Manipulation_Move_Robot::checkLocalization()
{
    return (robotPose_.pose.position.x != 0 || robotPose_.pose.position.y != 0 || robotPose_.pose.position.z != 0);
}

bool Suturo_Manipulation_Move_Robot::xCoordArrived(geometry_msgs::PoseStamped targetPose)
{
    return (robotPose_.pose.position.x < targetPose.pose.position.x + 0.01 && robotPose_.pose.position.x > targetPose.pose.position.x - 0.01);
}

bool Suturo_Manipulation_Move_Robot::yCoordArrived(geometry_msgs::PoseStamped targetPose)
{
    return (robotPose_.pose.position.y < targetPose.pose.position.y + 0.01 && robotPose_.pose.position.y > targetPose.pose.position.y - 0.01);
}

bool Suturo_Manipulation_Move_Robot::orientationArrived(tf::Quaternion robotOrientation, tf::Quaternion *targetOrientation)
{
    return (targetOrientation->angle(robotOrientation) < 0.05) && (targetOrientation->angle(robotOrientation) > -0.05);
}

bool Suturo_Manipulation_Move_Robot::calculateZTwist(tf::Quaternion *targetQuaternion)
{
    // TODO: Schöner machen

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
    ROS_ERROR_STREAM("Calculation of z twist failed, rotating and moving aborted!");
    return false;
}


bool Suturo_Manipulation_Move_Robot::rotateBase()
{

    tf::Quaternion *targetQuaternion = new tf::Quaternion(targetPoseBaseLink_.pose.orientation.x, targetPoseBaseLink_.pose.orientation.y, targetPoseBaseLink_.pose.orientation.z, targetPoseBaseLink_.pose.orientation.w);
    tf::Quaternion robotOrientation(0, 0, 0, 1);

    ROS_INFO("Begin to rotate base");

    if (calculateZTwist(targetQuaternion))
    {
        while (nh_->ok() && !orientationArrived(robotOrientation, targetQuaternion) && transformToBaseLink(targetPose_, targetPoseBaseLink_))
        {

            if (!getCollisions().empty())
            {
                ROS_ERROR_STREAM("Detect collision, rotating and moving aborted!");
                return false;
            }

            base_cmd_.angular.z = zTwist_;
            cmd_vel_pub_.publish(base_cmd_);

            targetQuaternion = new tf::Quaternion(targetPoseBaseLink_.pose.orientation.x, targetPoseBaseLink_.pose.orientation.y, targetPoseBaseLink_.pose.orientation.z, targetPoseBaseLink_.pose.orientation.w);
            ROS_INFO_STREAM(targetQuaternion->angle(robotOrientation));
        }
        return true;
    }
    else
    {
        return false;
    }
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

void Suturo_Manipulation_Move_Robot::subscriberCbLaserScan(const sensor_msgs::LaserScan& scan){
	collisions_.clear();
	for (int i = 0; i < scan.ranges.size(); i++){
		
		//cosinussatz um abstand zu mittelpunkt zu berechnen
		double alpha = scan.angle_increment*i + 0.872664626;
		double b = scan.ranges[i];
		double c = -0.275;//dist base_link to base_laser_link
		double a = sqrt((b*b) + (c*c) + (2*b*c * cos(alpha)));
		if (a < footprint_radius){
			//angle relativ to baselink
			//~ c = -c;
			double beta = acos ( (b*b - a*a - c*c) / (2*a*c) );
			if (alpha <= M_PI){
				beta = M_PI - beta;
			} else {
				beta += M_PI;
			}
			collisions_.push_back(beta);
			ROS_INFO_STREAM("alpha " << alpha);
			ROS_INFO_STREAM("beta " << beta);
		}
	}
	if (!collisions_.empty()){
		ROS_ERROR_STREAM("COLLISIONelf" );
	}
}

std::vector<double> Suturo_Manipulation_Move_Robot::getCollisions()
{
    return collisions_;
}

bool Suturo_Manipulation_Move_Robot::collisionInFront()
{

    for (int position = getCollisions().size() - 1; position > 0; position--)
    {
        // between 90 degrees and 270 degrees
        if (getCollisions().at(position) > 1.57 && getCollisions().at(position) < 4.71)
        {
            ROS_ERROR_STREAM("Collision in front!");
            return true;
        }
    }

    return false;
}

bool Suturo_Manipulation_Move_Robot::collisionOnRight()
{

    for (int position = getCollisions().size() - 1; position > 0; position--)
    {
        // wenn auf der rechten seite des robos 90 grad ist
        // between 45 degrees and 135 degrees
        if (getCollisions().at(position) > 0.79 && getCollisions().at(position) < 2.36)
        {
            ROS_ERROR_STREAM("Collision on right side!");
            return true;
        }
    }

    return false;
}

bool Suturo_Manipulation_Move_Robot::collisionOnLeft()
{

    for (int position = getCollisions().size() - 1; position > 0; position--)
    {
        // wenn auf der linken seite des robos 270 grad ist
        // between 225 degrees and 315 degrees
        if (getCollisions().at(position) > 3.93 && getCollisions().at(position) < 5.5)
        {
            ROS_ERROR_STREAM("Collision on left side!");
            return true;
        }
    }
    return false;
}

bool Suturo_Manipulation_Move_Robot::checkYVariation()
{
    double currentVariation = abs(targetPose_.pose.position.y - robotPose_.pose.position.y);

    return currentVariation <= yVariation_;
}

bool Suturo_Manipulation_Move_Robot::driveBase(geometry_msgs::PoseStamped targetPose)
{

    // TODO: Bennys Interpolator nutzen, um bei geringerer Zielentferung eine geringere Geschwindigkeit zu nutzen

    // reset values
    base_cmd_.linear.x = 0;
    base_cmd_.linear.y = 0;
    base_cmd_.angular.z = 0;

    targetPose_ = targetPose;

    if (checkCollision(targetPose_))
    {
        ROS_ERROR_STREAM("targetpose in collision!");
        return false;
    }

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
    // wenn y > 0 links vom robo ist und y < 0 rechts vom robo
    if (0 < targetPoseBaseLink_.pose.position.y && !collisionOnLeft())
    {
        yTwist = 0.1;
    }
    else if (0 > targetPoseBaseLink_.pose.position.y && !collisionOnRight())
    {
        yTwist = (-0.1);
    }
    else
    {
        yTwist = 0;
    }
    ROS_INFO_STREAM("yTwist: " << yTwist);

    // true, if the robot moved, false if not
    bool moved = true;

    // calculate current yVariation
    yVariation_ = abs(targetPose_.pose.position.y - robotPose_.pose.position.y);

    while (nh_->ok() && moved)
    {
        // reset all the values!!!!11elf
        base_cmd_.linear.x = 0;
        base_cmd_.linear.y = 0;
        base_cmd_.angular.z = 0;
        moved = false;

        // set x twist
        if (!xCoordArrived(targetPose) && targetPoseBaseLink_.pose.position.x > 0 && !collisionInFront())
        {
            base_cmd_.linear.x = 0.1;
            moved = true;
        }

        // set y twist
        if (!yCoordArrived(targetPose_) && yTwist != 0)
        {
            if (checkYVariation())
            {
                base_cmd_.linear.y = yTwist;
                moved = true;
            }
            else
            {
                yTwist = (yTwist * (-1));
                base_cmd_.linear.y = yTwist;
                moved = true;
            }
        }
        // publish twists to move
        cmd_vel_pub_.publish(base_cmd_);
        transformToBaseLink(targetPose_, targetPoseBaseLink_);
    }

    ROS_INFO_STREAM(targetPose_);
    ROS_INFO_STREAM(robotPose_);

    // Finish him!
    return true;
}
