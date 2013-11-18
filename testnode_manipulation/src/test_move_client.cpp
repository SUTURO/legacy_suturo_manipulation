#include "ros/ros.h"
#include "../../../../devel/include/testnode_manipulation/t_move_srv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t_move_client");
  if (argc != 6)
  {
    ROS_INFO("usage: t_move X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<testnode_manipulation::t_move_srv>("t_move_srv");
  testnode_manipulation::t_move_srv srv;
  srv.request.x = atof(argv[1]);
  srv.request.y = atof(argv[2]);
  srv.request.z = atof(argv[3]);
  srv.request.arm = argv[4];
  srv.request.extra = argv[5];
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.succ);
  }
  else
  {
    ROS_ERROR("Failed to call service t_move");
    return 1;
  }

  return 0;
}
