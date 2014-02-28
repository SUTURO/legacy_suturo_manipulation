/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperReleaseResult.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASERESULT_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASERESULT_H
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

#include "pr2_gripper_sensor_msgs/PR2GripperReleaseData.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperReleaseResult_ {
  typedef PR2GripperReleaseResult_<ContainerAllocator> Type;

  PR2GripperReleaseResult_()
  : data()
  {
  }

  PR2GripperReleaseResult_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperReleaseData_<ContainerAllocator>  _data_type;
   ::pr2_gripper_sensor_msgs::PR2GripperReleaseData_<ContainerAllocator>  data;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperReleaseResult
typedef  ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<std::allocator<void> > PR2GripperReleaseResult;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult> PR2GripperReleaseResultPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult const> PR2GripperReleaseResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b4b68d48ac7d07bdb11b7f3badfa9266";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb4b68d48ac7d07bdULL;
  static const uint64_t static_value2 = 0xb11b7f3badfa9266ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperReleaseResult";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result\n\
PR2GripperReleaseData data\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperReleaseData\n\
# the control state of our realtime controller\n\
PR2GripperSensorRTState rtstate\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperSensorRTState\n\
# the control state of our realtime controller\n\
int8 realtime_controller_state\n\
\n\
# predefined values to indicate our realtime_controller_state\n\
int8 DISABLED = 0\n\
int8 POSITION_SERVO = 3\n\
int8 FORCE_SERVO = 4\n\
int8 FIND_CONTACT = 5\n\
int8 SLIP_SERVO = 6\n\
";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperReleaseResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperReleaseResult_<ContainerAllocator> & v) 
  {
    s << indent << "data: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperReleaseData_<ContainerAllocator> >::stream(s, indent + "  ", v.data);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERRELEASERESULT_H

