#ifndef SUTURO_MANIPULATION_GRIPPER_CONTROLLER
#define SUTURO_MANIPULATION_GRIPPER_CONTROLLER

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <suturo_manipulation_msgs/RobotBodyPart.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;


class Gripper
{

private:
    GripperClient *gripper_client_;
    int connected_to_controller_;
    double gripper_state_;
    std::string arm_;

    static const std::string LEFT_ARM;
    static const std::string RIGHT_ARM;

    static const std::string R_GRIPPER_TOPIC;
    static const std::string L_GRIPPER_TOPIC;

    /**
     * Feedback collback function that saves the gripper state during a servercall, in
     * case the server doesn't responde in time.
     */
    void feedbackCb(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr &feedback);

public:
    static const double GRIPPER_MAX_POSITION = 0.09;
    static const double GRIPPER_MIN_POSITION = 0.0;

    static const double GRIPPER_DEPTH = 0.05;

    static const double R_GRIPPER_PALM_LENGTH = 0.14; //this + depth should be 19cm
    static const double L_GRIPPER_PALM_LENGTH = 0.175; //this + depth should be 22,5cm


    Gripper(std::string arm);

    ~Gripper();

    /**
     * Says wether or not there is a connection to the gripper controller.
     *
     * @return 1, if connected
     *                  0, otherwise
     */
    int is_connected_to_controller()
    {
        return connected_to_controller_;
    }

    /**
     * Closes the Gripper.
     *
     * @return the width the gripper is open
     */
    double close_gripper(double force = -1);

    /**
     * Opens the Gripper.
     *
     * @return the width the gripper is open
     */
    double open_gripper(double force = 20);

    double get_gripper_palm_length();

    std::string get_gripper_group();

    std::vector<std::string> get_gripper_links();

    /**
     * Returns a list of all right gripper links.
     */
    static const std::vector<std::string> get_r_gripper_links()
    {
        static std::vector<std::string> r2;
        r2.push_back("r_gripper_l_finger_link");
        r2.push_back("r_gripper_l_finger_tip_link");
        r2.push_back("r_gripper_motor_accelerometer_link");
        r2.push_back("r_gripper_palm_link");
        r2.push_back("r_gripper_r_finger_link");
        r2.push_back("r_gripper_r_finger_tip_link");
        static const std::vector<std::string> r = r2;
        return r;
    }

    /**
    * Returns a list of all right gripper links.
    */
    static const std::vector<std::string> get_l_gripper_links()
    {
        static std::vector<std::string> r2;
        r2.push_back("l_gripper_l_finger_link");
        r2.push_back("l_gripper_l_finger_tip_link");
        r2.push_back("l_gripper_motor_accelerometer_link");
        r2.push_back("l_gripper_palm_link");
        r2.push_back("l_gripper_r_finger_link");
        r2.push_back("l_gripper_r_finger_tip_link");
        r2.push_back("l_force_torque_adapter_link");
        r2.push_back("l_force_torque_link");
        static const std::vector<std::string> r = r2;
        return r;
    }

    static const std::string get_r_group_name(){
        return "right_gripper";
    }

    static const std::string get_l_group_name(){
        return "left_gripper";
    }

};

#endif
