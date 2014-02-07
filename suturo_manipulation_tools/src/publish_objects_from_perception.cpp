#include "ros/ros.h"
#include "suturo_perception_msgs/GetClusters.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <shape_tools/solid_primitive_dims.h>
#include <suturo_manipulation_planning_scene_interface.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

	Suturo_Manipulation_Planning_Scene_Interface pi(&n);

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetClusters>("/suturo/GetClusters");
  suturo_perception_msgs::GetClusters clusterSrv;
  clusterSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  moveit_msgs::CollisionObject co;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);  
  while(true)
  {
    if (clusterClient.call(clusterSrv))
    {
      ROS_INFO("Cluster Service call successful");
      ROS_INFO("List size: %ld", (long int)clusterSrv.response.perceivedObjs.size() );

      // wait a sec if list is empty. service may not be ready yet
      if((long int)clusterSrv.response.perceivedObjs.size() == 0)
      {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
        ROS_INFO_STREAM("No objects perceived yet.");
        continue;
      }

      for(int i=0; i < clusterSrv.response.perceivedObjs.size(); i++ ) {
				
				suturo_perception_msgs::Cuboid cubi = clusterSrv.response.perceivedObjs[i].matched_cuboid;
				
				//create object
				co.header.stamp = ros::Time::now();
				co.header.frame_id = clusterSrv.response.perceivedObjs[i].frame_id;
				//~ std::string  a = itoa(i);
				co.id = "obj";
				co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = cubi.length1;
				co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = cubi.length2;
				co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = cubi.length3;
				co.primitive_poses[0] = cubi.pose;
				
				pi.addObject(co);
				
        //~ std::stringstream vfh_ss;
        //~ for (int j = 0; j < clusterSrv.response.perceivedObjs[i].c_vfh_estimation.size(); j++)
        //~ {
          //~ vfh_ss << clusterSrv.response.perceivedObjs[i].c_vfh_estimation.at(j);
          //~ vfh_ss << " ";
        //~ }
        //~ ROS_INFO("VFH Estimation: %s ", vfh_ss.str().c_str());
        //~ ROS_INFO("SVM Result: %s ", clusterSrv.response.perceivedObjs[i].c_svm_result.c_str());
        //~ ROS_INFO("Pose: %d ", clusterSrv.response.perceivedObjs[i].c_pose);
      }
      ROS_INFO_STREAM("------------------------------------------------------------");
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetClusters");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
