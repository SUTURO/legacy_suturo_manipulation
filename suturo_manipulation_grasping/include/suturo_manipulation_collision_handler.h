#ifndef SUTURO_MANIPULATION_COLLISION_HANDLER
#define SUTURO_MANIPULATION_COLLISION_HANDLER
#include <ros/ros.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <tf/transform_listener.h>

using namespace std;
class Collision_Handler
{
  protected:
    int attempt_, maxAttempts_;
    int *collisionValues_;
    bool rightArm_;
    Suturo_Manipulation_Planning_Scene_Interface* pi_;
    tf::TransformListener* listener_;
    ros::Publisher vis_pub_;
    void checkForPreviousCollision(int yValue, int z_value, moveit_msgs::CollisionObject& co);
    void publishMarker(geometry_msgs::PoseStamped pose);

  public:
    Collision_Handler(ros::NodeHandle* nh, int maxAttempts, Suturo_Manipulation_Planning_Scene_Interface* pi);
    void reset(bool rightArm);
    void handleCollision(int collisionValue, moveit_msgs::CollisionObject& co);
    bool attemptValid();

};

#endif
