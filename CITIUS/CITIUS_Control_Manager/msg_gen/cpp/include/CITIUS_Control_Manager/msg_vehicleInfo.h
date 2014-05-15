/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Manager/msg/msg_vehicleInfo.msg */
#ifndef CITIUS_CONTROL_MANAGER_MESSAGE_MSG_VEHICLEINFO_H
#define CITIUS_CONTROL_MANAGER_MESSAGE_MSG_VEHICLEINFO_H
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


namespace CITIUS_Control_Manager
{
template <class ContainerAllocator>
struct msg_vehicleInfo_ {
  typedef msg_vehicleInfo_<ContainerAllocator> Type;

  msg_vehicleInfo_()
  : test(0)
  {
  }

  msg_vehicleInfo_(const ContainerAllocator& _alloc)
  : test(0)
  {
  }

  typedef uint8_t _test_type;
  uint8_t test;


  typedef boost::shared_ptr< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_vehicleInfo
typedef  ::CITIUS_Control_Manager::msg_vehicleInfo_<std::allocator<void> > msg_vehicleInfo;

typedef boost::shared_ptr< ::CITIUS_Control_Manager::msg_vehicleInfo> msg_vehicleInfoPtr;
typedef boost::shared_ptr< ::CITIUS_Control_Manager::msg_vehicleInfo const> msg_vehicleInfoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace CITIUS_Control_Manager

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1f6e6cd7d3f3bd76dda35fffbcf752ff";
  }

  static const char* value(const  ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1f6e6cd7d3f3bd76ULL;
  static const uint64_t static_value2 = 0xdda35fffbcf752ffULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Manager/msg_vehicleInfo";
  }

  static const char* value(const  ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 test\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.test);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_vehicleInfo_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::CITIUS_Control_Manager::msg_vehicleInfo_<ContainerAllocator> & v) 
  {
    s << indent << "test: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.test);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CITIUS_CONTROL_MANAGER_MESSAGE_MSG_VEHICLEINFO_H
