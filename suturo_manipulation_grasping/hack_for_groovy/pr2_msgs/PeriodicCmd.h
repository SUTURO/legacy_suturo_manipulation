/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-common-1.10.3/debian/ros-groovy-pr2-common/opt/ros/groovy/stacks/pr2_common/pr2_msgs/msg/PeriodicCmd.msg */
#ifndef PR2_MSGS_MESSAGE_PERIODICCMD_H
#define PR2_MSGS_MESSAGE_PERIODICCMD_H
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

namespace pr2_msgs
{
template <class ContainerAllocator>
struct PeriodicCmd_ {
  typedef PeriodicCmd_<ContainerAllocator> Type;

  PeriodicCmd_()
  : header()
  , profile()
  , period(0.0)
  , amplitude(0.0)
  , offset(0.0)
  {
  }

  PeriodicCmd_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , profile(_alloc)
  , period(0.0)
  , amplitude(0.0)
  , offset(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _profile_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  profile;

  typedef double _period_type;
  double period;

  typedef double _amplitude_type;
  double amplitude;

  typedef double _offset_type;
  double offset;


  typedef boost::shared_ptr< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_msgs::PeriodicCmd_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PeriodicCmd
typedef  ::pr2_msgs::PeriodicCmd_<std::allocator<void> > PeriodicCmd;

typedef boost::shared_ptr< ::pr2_msgs::PeriodicCmd> PeriodicCmdPtr;
typedef boost::shared_ptr< ::pr2_msgs::PeriodicCmd const> PeriodicCmdConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_msgs::PeriodicCmd_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_msgs::PeriodicCmd_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "95ab7e548e3d4274f83393129dd96c2e";
  }

  static const char* value(const  ::pr2_msgs::PeriodicCmd_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x95ab7e548e3d4274ULL;
  static const uint64_t static_value2 = 0xf83393129dd96c2eULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_msgs/PeriodicCmd";
  }

  static const char* value(const  ::pr2_msgs::PeriodicCmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# This message is used to set the parameters of a profile executed by the\n\
# laser tilt controller.\n\
Header header\n\
string profile\n\
float64 period\n\
float64 amplitude\n\
float64 offset\n\
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
";
  }

  static const char* value(const  ::pr2_msgs::PeriodicCmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pr2_msgs::PeriodicCmd_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.profile);
    stream.next(m.period);
    stream.next(m.amplitude);
    stream.next(m.offset);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PeriodicCmd_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_msgs::PeriodicCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_msgs::PeriodicCmd_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "profile: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.profile);
    s << indent << "period: ";
    Printer<double>::stream(s, indent + "  ", v.period);
    s << indent << "amplitude: ";
    Printer<double>::stream(s, indent + "  ", v.amplitude);
    s << indent << "offset: ";
    Printer<double>::stream(s, indent + "  ", v.offset);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_MSGS_MESSAGE_PERIODICCMD_H

