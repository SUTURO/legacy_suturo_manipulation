#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <suturo_manipulation_gripper_controller.h>
#include <suturo_manipulation_grasping.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <suturo_manipulation_move_robot.h>
#include <sensor_msgs/LaserScan.h>
#include <suturo_manipulation_grasping_reactive.h>
#include <suturo_manipulation_mesh_loader.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void putObjects(ros::Publisher pub_co)
{
    //real
    //~ double tischposiZ = 0.86;
    //gazebo
    //~ roslaunch pr2_teleop_general pr2_teleop_general_keyboard_bodyhead_only.launch
    double tischposiZ = 0.57;

    ros::WallDuration(1.0).sleep();

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "/base_footprint";

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitive_poses.resize(1);

    co.id = "dlink";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add box2
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.035 ;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.24;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.169;
    co.primitive_poses[0].position.x = 0.67;
    co.primitive_poses[0].position.y = -0.4;
    co.primitive_poses[0].position.z = tischposiZ + 0.03 + 0.085;
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, M_PI_2, M_PI_2);

    pub_co.publish(co);

    // remove table
    co.id = "table";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add table
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.75;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.8;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tischposiZ;
    co.primitive_poses[0].position.x = 0.85;
    co.primitive_poses[0].position.y = 0;
    co.primitive_poses[0].position.z = tischposiZ / 2;
    co.primitive_poses[0].orientation.x = 0;
    co.primitive_poses[0].orientation.y = 0;
    co.primitive_poses[0].orientation.z = 0;
    co.primitive_poses[0].orientation.w = 1;
    pub_co.publish(co);

    // remove table
    co.id = "table2";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add table
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.95;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.45;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    co.primitive_poses[0].position.x = 0.85;
    co.primitive_poses[0].position.y = 0;

    //  co.primitive_poses[0].position.z = tischposiZ+0.015;
    // co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    //  pub_co.publish(co);

    co.primitive_poses[0].position.z = tischposiZ + 0.015;
    co.primitive_poses[0].orientation.x = 0;
    co.primitive_poses[0].orientation.y = 0;
    co.primitive_poses[0].orientation.z = 0;
    co.primitive_poses[0].orientation.w = 1;
    pub_co.publish(co);

    // remove box1
    co.id = "cafetfilter";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    // add box1
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.057;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.132;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.197;
    co.primitive_poses[0].position.x = 0.67;
    co.primitive_poses[0].position.y = 0.4;
    co.primitive_poses[0].position.z = tischposiZ + 0.03 + 0.099; //tischposiZ + 0.08;
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI_2);

    pub_co.publish(co);

    co.id = "beer2";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.25;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.0375;

    co.primitive_poses[0].position.x = 0.67;
    co.primitive_poses[0].position.y = 0;
    co.primitive_poses[0].position.z = tischposiZ + 0.03 + 0.126;
    co.primitive_poses[0].orientation.x = 0;
    co.primitive_poses[0].orientation.y = 0;
    co.primitive_poses[0].orientation.z = 0;
    co.primitive_poses[0].orientation.w = 1;
    pub_co.publish(co);

    ros::WallDuration(1.0).sleep();
}

void openhand()
{

    Gripper g;
    g.open_l_gripper();
    g.close_l_gripper();
}

void subscriberCb(const sensor_msgs::LaserScan &scan)
{
    //~ sensor_msgs::LaserScan base_scan;
    //~ listener_.transformPose("/base_link", scan, base_scan);
    for (int i = 0; i < scan.ranges.size(); i++)
    {


        double alpha = scan.angle_increment * i + 0.872664626;
        double b = scan.ranges[i];
        double c = -0.275;//dist base_link to base_laser_link
        double a = sqrt((b * b) + (c * c) + (2 * b * c * cos(alpha)));
        //~ ROS_INFO_STREAM((alpha * 180 / M_PI) << " a: " << a << " b: " << b);
        if (a < 0.5)
        {
            ROS_WARN_STREAM(i << "zu nah!!!!");
            //~ inCollision_ = true;
            return;
        }
    }

}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "right_arm_pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    Mesh_loader ml;
    ROS_INFO_STREAM(ml.load_corny());
    ROS_INFO_STREAM("\n\n");
    // ROS_INFO_STREAM(ml.load_pringles());

    // Suturo_Manipulation_Planning_Scene_Interface pi(&nh);
    // Grasping* g = new Grasping_reactive(&pi);
    // g->drop("asd");

//     ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
// ros::WallDuration(1.0).sleep();
//     moveit_msgs::CollisionObject co;
//     co.header.stamp = ros::Time::now();
//     co.header.frame_id = "/base_footprint";
//     co.id = "corny";
//     co.operation = moveit_msgs::CollisionObject::ADD;

//     co.meshes.resize(1);
//     co.meshes[0] = ml.load_pringles();
//     co.mesh_poses.resize(1);
//     co.mesh_poses[0].position.x = 2;
//     co.mesh_poses[0].position.y = 1;
//     co.mesh_poses[0].position.z = 2;
//     co.mesh_poses[0].orientation.w = 1;
//     ros::WallDuration(1.0).sleep();
//     pub_co.publish(co);
//     ros::WallDuration(1.0).sleep();
    // putObjects(pub_co);

    // Gripper g;

    // if (argc == 2){
    //  g.open_l_gripper();
    // } else if (argc == 3) {
    //  g.close_l_gripper();
    // }
    //~ geometry_msgs::PoseStamped p;
    //~ p.header.frame_id = "/base_footprint";
    //~ p.pose.position.x = 0.40;
    //~ p.pose.position.y = 0;
    //~ p.pose.position.z = 0.625;
    //~ p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    // ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    // putObjects(pub_co);

    // Suturo_Manipulation_Planning_Scene_Interface pi(&nh);
    // pi.allowCollision("r_gripper_l_finger_link", "corny");
    // pi.allowCollision("r_gripper_l_finger_tip_link", "corny");
    // pi.allowCollision("r_gripper_motor_accelerometer_link", "corny");
    // pi.allowCollision("r_gripper_palm_link", "corny");
    // pi.allowCollision("r_gripper_r_finger_link", "corny");
    // pi.allowCollision("r_gripper_r_finger_tip_link", "corny");

    // pi.allowCollision("l_gripper_l_finger_link", "cafetfilter");
    // pi.allowCollision("l_gripper_l_finger_tip_link", "cafetfilter");
    // pi.allowCollision("l_gripper_motor_accelerometer_link", "cafetfilter");
    // pi.allowCollision("l_gripper_palm_link", "cafetfilter");
    // pi.allowCollision("l_gripper_r_finger_link", "cafetfilter");
    // pi.allowCollision("l_gripper_r_finger_tip_link", "cafetfilter");

    // moveit_msgs::PlanningScene ps;
    // ROS_INFO_STREAM(pi.getPlanningScene(ps));
    // ROS_INFO_STREAM(ps);
    // ps.robot_state.multi_dof_joint_state.joint_transforms[0].translation.x = 1;

    // geometry_msgs::PoseStamped targetPose;
    // targetPose.header.frame_id = "/odom_combined";
    // targetPose.pose.position.x = atof(argv[1]);
    // targetPose.pose.position.y = atof(argv[2]);
    // targetPose.pose.position.z = atof(argv[3]);
    // targetPose.pose.orientation.w = 1;

    // Suturo_Manipulation_Move_Robot moveRobot(&nh);
    // ROS_INFO_STREAM("collision: " << (moveRobot.checkFullCollision(targetPose)));

    //~ ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    //~ putObjects(pub_co);
    //~
    //~ Gripper g;
    //~
    //~ if (argc == 2){
    //~ g.open_l_gripper();
    //~ } else if (argc == 3) {
    //~ g.close_l_gripper();
    //~ }

    //~ move_group_interface::MoveGroup group("right_arm");
    //~ geometry_msgs::PoseStamped p;
    //~ p.header.frame_id = "/base_footprint";
    //~ p.pose.position.x = 0.40;
    //~ p.pose.position.y = 0;
    //~ p.pose.position.z = 0.625;
    //~ p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    //~ group.setPoseTarget(p);
    //~ group.move();

    //~ move_group_interface::MoveGroup group("left_gripper");
    //~
    //~ std::vector<std::string> bla1 = group.getJoints();
    //~ for (int i = 0; i < bla1.size(); i++) ROS_INFO_STREAM(bla1.at(i));
    //~
    //~ std::vector<double> bla = group.getCurrentJointValues();
    //~
    //~ for (int i = 0; i < bla.size(); i++) ROS_INFO_STREAM(bla.at(i));
    //~
    //~ openhand();
    //~
    //~ bla = group.getCurrentJointValues();
    //~ for (int i = 0; i < bla.size(); i++) ROS_INFO_STREAM(bla.at(i));
    //~

    //~ Suturo_Manipulation_Planning_Scene_Interface pi(&nh);

    //~ Grasping grasper(&pi);
    //~ grasper.drop(suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM);
    //~ grasper.pick("dlink", suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
    //~ grasper.pick("dlink", suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
    //~ grasper.pick("corny", suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
    //~ ROS_INFO("done.");
    //~ grasper.pick("corny", suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM);
    //~ ROS_INFO("done.");

    //~ move_group_interface::MoveGroup group("left_arm");
    //~ geometry_msgs::PoseStamped p;
    //~ p.header.frame_id = "/base_footprint";
    //~ p.pose.position.x = 0.5;
    //~ p.pose.position.y = 0.2;
    //~ p.pose.position.z = 1;
    //~ p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, -M_PI_2);
    //~ group.setPoseTarget(p);

    //~ tf::Quaternion q;
    //~ double roll, pitch, yaw;
    //~ tf::quaternionMsgToTF(p.pose.orientation, q);
    //~ ROS_INFO_STREAM(p.pose.orientation);
    //~ tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //~ ROS_INFO("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);

    //~ if (!group.move()) return 0;
    //~ ROS_INFO("done.");

    //~ grasper.pick("corny", suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
    //~ ROS_INFO("done.");

    //~ ROS_INFO("done.");
    //~ grasper.pick("corny", suturo_manipulation_msgs::RobotBodyPart::LEFT_ARM);
    //~ grasper.drop(suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
    //~ grasper.drop(suturo_manipulation_msgs::RobotBodyPart::RIGHT_ARM);
    //~ grasper.drop("corny");
    //~ openhand();

    //~ moveit_msgs::PlanningScene ps;
    //~ pi.getPlanningScene(ps);
    //~ ROS_INFO_STREAM("ps: " << ps);

    //std::vector<moveit_msgs::AttachedCollisionObject> muh = pi.getAttachedObjects();
    //ROS_INFO_STREAM("objects " << muh.at(0));
    //~ grasper.drop("box2");

    //geometry_msgs::PoseStamped p;
    //p.header.frame_id = "/base_footprint";
    //p.pose.position.x = 0.65;
    //p.pose.position.y = -0.3;
    //p.pose.position.z = 0.821;
    //p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI_2, M_PI_4);
    //grasper.l_arm_pick("box2");

    //move_group_interface::MoveGroup group("right_arm");
    //group.setPlanningTime(45.0);
    /*pick(group);*/

    /*geometry_msgs::PoseStamped p;
    p.header.frame_id = "/base_footprint";
    p.pose.position.x = 0.59;
    p.pose.position.y = 0;
    p.pose.position.z = 0.625;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;*/

    //group.setPoseTarget(p);
    //group.move();


    //move_group_interface::MoveGroup group("right_arm");
    //pick(group);
    //Gripper g;
    //pi.attachObject("box1", "r_wrist_roll_link", Gripper::get_r_gripper_links());
    //moveit_msgs::PlanningScene ps;
    //pi.getPlanningScene(ps);
    //std::vector<moveit_msgs::AttachedCollisionObject> muh = pi.getAttachedObjects();
    //ROS_INFO_STREAM("objects " << muh.at(0));


    //pi.detachObject("box1");
    //muh = pi.getAttachedObjects();
    //ROS_INFO_STREAM("objects " << muh.at(0));

    //ROS_INFO_STREAM("ps " << ps.robot_state);
    //ros::Publisher pub2 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    //pub2.publish(ps);

    //ROS_INFO_STREAM("dsads  " << ps);

    ROS_DEBUG_STREAM("finish");
    ROS_INFO_STREAM("finish");
    ros::waitForShutdown();
    return 0;
}
