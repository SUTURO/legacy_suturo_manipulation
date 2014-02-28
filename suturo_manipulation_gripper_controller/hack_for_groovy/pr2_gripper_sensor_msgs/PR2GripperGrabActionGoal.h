/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperGrabActionGoal.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERGRABACTIONGOAL_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERGRABACTIONGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "pr2_gripper_sensor_msgs/PR2GripperGrabGoal.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperGrabActionGoal_ {
  typedef PR2GripperGrabActionGoal_<ContainerAllocator> Type;

  PR2GripperGrabActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  PR2GripperGrabActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperGrabGoal_<ContainerAllocator>  _goal_type;
   ::pr2_gripper_sensor_msgs::PR2GripperGrabGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperGrabActionGoal
typedef  ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<std::allocator<void> > PR2GripperGrabActionGoal;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal> PR2GripperGrabActionGoalPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal const> PR2GripperGrabActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e540762b3e9da0e115b0380658bdbe12";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe540762b3e9da0e1ULL;
  static const uint64_t static_value2 = 0x15b0380658bdbe12ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperGrabActionGoal";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
PR2GripperGrabGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperGrabGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal\n\
PR2GripperGrabCommand command\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperGrabCommand\n\
# The gain to use to evaluate how hard an object should be\n\
# grasped after it is contacted. This is based on hardness\n\
# estimation as outlined in TRO paper (see wiki).\n\
# \n\
# Try 0.03\n\
#\n\
# Units (N/(m/s^2))\n\
float64 hardness_gain\n\
\n\
";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperGrabActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperGrabActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperGrabGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERGRABACTIONGOAL_H

