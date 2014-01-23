#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>

namespace suturo{

    class Head_Controller: public pr2_controller_interface::Controller
    {
        private:
            // Joint torques
            KDL::JntArray  tau_;          

            // The chain of links and joints
            pr2_mechanism_model::Chain chain_;
            pr2_mechanism_model::RobotState *robot_;           

            ros::Publisher vis_pub;
            
            tf::TransformListener listener;
            
            geometry_msgs::PointStamped goalPoint_;
            geometry_msgs::PointStamped originPoint_;
            
            bool updated;

            ros::Subscriber sub_;

            control_toolbox::Pid pid_controller_;
            ros::Time time_of_last_cycle_;
            double pid_error_;

        public:
            bool init(pr2_mechanism_model::RobotState *robot,
                    ros::NodeHandle &n);
            void starting();
            void update();
            void stopping();
        private:
            void setGoalCB(geometry_msgs::PoseStamped msg);
            bool switchControllers(ros::NodeHandle &n);
    };
}
