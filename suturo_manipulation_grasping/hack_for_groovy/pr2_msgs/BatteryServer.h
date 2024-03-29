/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-pr2-common-1.10.3/debian/ros-groovy-pr2-common/opt/ros/groovy/stacks/pr2_common/pr2_msgs/msg/BatteryServer.msg */
#ifndef PR2_MSGS_MESSAGE_BATTERYSERVER_H
#define PR2_MSGS_MESSAGE_BATTERYSERVER_H
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
#include "pr2_msgs/BatteryState.h"

namespace pr2_msgs
{
template <class ContainerAllocator>
struct BatteryServer_ {
  typedef BatteryServer_<ContainerAllocator> Type;

  BatteryServer_()
  : header()
  , id(0)
  , lastTimeSystem(0)
  , timeLeft(0)
  , averageCharge(0)
  , message()
  , lastTimeController(0)
  , present(0)
  , charging(0)
  , discharging(0)
  , reserved(0)
  , powerPresent(0)
  , powerNG(0)
  , inhibited(0)
  , battery()
  {
  }

  BatteryServer_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , id(0)
  , lastTimeSystem(0)
  , timeLeft(0)
  , averageCharge(0)
  , message(_alloc)
  , lastTimeController(0)
  , present(0)
  , charging(0)
  , discharging(0)
  , reserved(0)
  , powerPresent(0)
  , powerNG(0)
  , inhibited(0)
  , battery(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _id_type;
  int32_t id;

  typedef int32_t _lastTimeSystem_type;
  int32_t lastTimeSystem;

  typedef uint16_t _timeLeft_type;
  uint16_t timeLeft;

  typedef uint16_t _averageCharge_type;
  uint16_t averageCharge;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  message;

  typedef int32_t _lastTimeController_type;
  int32_t lastTimeController;

  typedef uint16_t _present_type;
  uint16_t present;

  typedef uint16_t _charging_type;
  uint16_t charging;

  typedef uint16_t _discharging_type;
  uint16_t discharging;

  typedef uint16_t _reserved_type;
  uint16_t reserved;

  typedef uint16_t _powerPresent_type;
  uint16_t powerPresent;

  typedef uint16_t _powerNG_type;
  uint16_t powerNG;

  typedef uint16_t _inhibited_type;
  uint16_t inhibited;

  typedef std::vector< ::pr2_msgs::BatteryState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pr2_msgs::BatteryState_<ContainerAllocator> >::other >  _battery_type;
  std::vector< ::pr2_msgs::BatteryState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pr2_msgs::BatteryState_<ContainerAllocator> >::other >  battery;

  enum { MAX_BAT_COUNT = 4 };
  enum { MAX_BAT_REG = 48 };

  typedef boost::shared_ptr< ::pr2_msgs::BatteryServer_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_msgs::BatteryServer_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BatteryServer
typedef  ::pr2_msgs::BatteryServer_<std::allocator<void> > BatteryServer;

typedef boost::shared_ptr< ::pr2_msgs::BatteryServer> BatteryServerPtr;
typedef boost::shared_ptr< ::pr2_msgs::BatteryServer const> BatteryServerConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pr2_msgs::BatteryServer_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pr2_msgs::BatteryServer_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pr2_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pr2_msgs::BatteryServer_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pr2_msgs::BatteryServer_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pr2_msgs::BatteryServer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4f6d6e54c9581beb1df7ea408c0727be";
  }

  static const char* value(const  ::pr2_msgs::BatteryServer_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4f6d6e54c9581bebULL;
  static const uint64_t static_value2 = 0x1df7ea408c0727beULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_msgs::BatteryServer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pr2_msgs/BatteryServer";
  }

  static const char* value(const  ::pr2_msgs::BatteryServer_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pr2_msgs::BatteryServer_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# DEPRECATED. Use pr2_msgs/BatteryServer2 instead.\n\
Header header\n\
uint32 MAX_BAT_COUNT=4\n\
uint32 MAX_BAT_REG=48\n\
int32 id  # unique ID for each battery server\n\
# Battery System Stats\n\
int32 lastTimeSystem #epoch time\n\
uint16 timeLeft      # in minutes\n\
uint16 averageCharge # in percent\n\
string message\n\
# Battery Controller Flags\n\
int32 lastTimeController #epoch time\n\
uint16 present\n\
uint16 charging\n\
uint16 discharging\n\
uint16 reserved\n\
uint16 powerPresent\n\
uint16 powerNG\n\
uint16 inhibited\n\
# for each battery\n\
pr2_msgs/BatteryState[] battery\n\
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
MSG: pr2_msgs/BatteryState\n\
# DEPRECATED.  Use pr2_msgs/BatteryState2 instead.\n\
# Each batteries registers\n\
int32 lastTimeBattery #epoch time\n\
uint16[48] batReg\n\
uint16[48] batRegFlag\n\
int32[48] batRegTime\n\
\n\
";
  }

  static const char* value(const  ::pr2_msgs::BatteryServer_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pr2_msgs::BatteryServer_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pr2_msgs::BatteryServer_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pr2_msgs::BatteryServer_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.id);
    stream.next(m.lastTimeSystem);
    stream.next(m.timeLeft);
    stream.next(m.averageCharge);
    stream.next(m.message);
    stream.next(m.lastTimeController);
    stream.next(m.present);
    stream.next(m.charging);
    stream.next(m.discharging);
    stream.next(m.reserved);
    stream.next(m.powerPresent);
    stream.next(m.powerNG);
    stream.next(m.inhibited);
    stream.next(m.battery);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BatteryServer_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_msgs::BatteryServer_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pr2_msgs::BatteryServer_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "lastTimeSystem: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lastTimeSystem);
    s << indent << "timeLeft: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.timeLeft);
    s << indent << "averageCharge: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.averageCharge);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
    s << indent << "lastTimeController: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lastTimeController);
    s << indent << "present: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.present);
    s << indent << "charging: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.charging);
    s << indent << "discharging: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.discharging);
    s << indent << "reserved: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.reserved);
    s << indent << "powerPresent: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.powerPresent);
    s << indent << "powerNG: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.powerNG);
    s << indent << "inhibited: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.inhibited);
    s << indent << "battery[]" << std::endl;
    for (size_t i = 0; i < v.battery.size(); ++i)
    {
      s << indent << "  battery[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pr2_msgs::BatteryState_<ContainerAllocator> >::stream(s, indent + "    ", v.battery[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PR2_MSGS_MESSAGE_BATTERYSERVER_H

