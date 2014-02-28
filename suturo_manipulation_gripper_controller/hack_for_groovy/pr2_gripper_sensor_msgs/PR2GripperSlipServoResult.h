/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-object-manipulation-0.7.5/debian/ros-groovy-pr2-object-manipulation/opt/ros/groovy/stacks/pr2_object_manipulation/manipulation/pr2_gripper_sensor_msgs/msg/PR2GripperSlipServoResult.msg */
#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVORESULT_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVORESULT_H
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

#include "pr2_gripper_sensor_msgs/PR2GripperSlipServoData.h"

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperSlipServoResult_ {
  typedef PR2GripperSlipServoResult_<ContainerAllocator> Type;

  PR2GripperSlipServoResult_()
  : data()
  {
  }

  PR2GripperSlipServoResult_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoData_<ContainerAllocator>  _data_type;
   ::pr2_gripper_sensor_msgs::PR2GripperSlipServoData_<ContainerAllocator>  data;


  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PR2GripperSlipServoResult
typedef  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<std::allocator<void> > PR2GripperSlipServoResult;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult> PR2GripperSlipServoResultPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult const> PR2GripperSlipServoResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1b10af616c7e94f609790b12cde04c6d";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1b10af616c7e94f6ULL;
  static const uint64_t static_value2 = 0x09790b12cde04c6dULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_gripper_sensor_msgs/PR2GripperSlipServoResult";
  }

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
#result\n\
PR2GripperSlipServoData data\n\
\n\
\n\
================================================================================\n\
MSG: pr2_gripper_sensor_msgs/PR2GripperSlipServoData\n\
# time the data was recorded at\n\
time stamp\n\
\n\
# the amount of deformation from action start (in meters)\n\
float64 deformation\n\
\n\
# the force experinced by the finger Pads  (N)\n\
# NOTE:this ignores data from the edges of the finger pressure\n\
float64 left_fingertip_pad_force\n\
float64 right_fingertip_pad_force\n\
\n\
# the current virtual parallel joint effort of the gripper (in N)\n\
float64 joint_effort\n\
\n\
# true if the object recently slipped\n\
bool slip_detected\n\
\n\
# true if we are at or exceeding the deformation limit\n\
# (see wiki page and param server for more info)\n\
bool deformation_limit_reached\n\
\n\
# true if we are at or exceeding our force \n\
# (see wiki page and param server for more info)\n\
bool fingertip_force_limit_reached\n\
\n\
# true if the controller thinks the gripper is empty\n\
# (see wiki page for more info)\n\
bool gripper_empty\n\
\n\
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

  static const char* value(const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PR2GripperSlipServoResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoResult_<ContainerAllocator> & v) 
  {
    s << indent << "data: ";
s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoData_<ContainerAllocator> >::stream(s, indent + "  ", v.data);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVORESULT_H

