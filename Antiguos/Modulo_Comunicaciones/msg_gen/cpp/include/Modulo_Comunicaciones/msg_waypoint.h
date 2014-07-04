/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Modulo_Comunicaciones/msg/msg_waypoint.msg */
#ifndef MODULO_COMUNICACIONES_MESSAGE_MSG_WAYPOINT_H
#define MODULO_COMUNICACIONES_MESSAGE_MSG_WAYPOINT_H
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


namespace Modulo_Comunicaciones
{
template <class ContainerAllocator>
struct msg_waypoint_ {
  typedef msg_waypoint_<ContainerAllocator> Type;

  msg_waypoint_()
  : waypoints()
  {
  }

  msg_waypoint_(const ContainerAllocator& _alloc)
  : waypoints(_alloc)
  {
  }

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _waypoints_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  waypoints;


  typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_waypoint
typedef  ::Modulo_Comunicaciones::msg_waypoint_<std::allocator<void> > msg_waypoint;

typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_waypoint> msg_waypointPtr;
typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_waypoint const> msg_waypointConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Modulo_Comunicaciones

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cb3f2bf066882d854358a8deef3110e6";
  }

  static const char* value(const  ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcb3f2bf066882d85ULL;
  static const uint64_t static_value2 = 0x4358a8deef3110e6ULL;
};

template<class ContainerAllocator>
struct DataType< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Modulo_Comunicaciones/msg_waypoint";
  }

  static const char* value(const  ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32[] waypoints\n\
";
  }

  static const char* value(const  ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.waypoints);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_waypoint_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Modulo_Comunicaciones::msg_waypoint_<ContainerAllocator> & v) 
  {
    s << indent << "waypoints[]" << std::endl;
    for (size_t i = 0; i < v.waypoints.size(); ++i)
    {
      s << indent << "  waypoints[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.waypoints[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // MODULO_COMUNICACIONES_MESSAGE_MSG_WAYPOINT_H
