/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Modulo_Comunicaciones/msg/msg_modo.msg */
#ifndef MODULO_COMUNICACIONES_MESSAGE_MSG_MODO_H
#define MODULO_COMUNICACIONES_MESSAGE_MSG_MODO_H
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
struct msg_modo_ {
  typedef msg_modo_<ContainerAllocator> Type;

  msg_modo_()
  : modo(0)
  {
  }

  msg_modo_(const ContainerAllocator& _alloc)
  : modo(0)
  {
  }

  typedef uint8_t _modo_type;
  uint8_t modo;


  typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_modo
typedef  ::Modulo_Comunicaciones::msg_modo_<std::allocator<void> > msg_modo;

typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_modo> msg_modoPtr;
typedef boost::shared_ptr< ::Modulo_Comunicaciones::msg_modo const> msg_modoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Modulo_Comunicaciones

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bc4ecbd89e2dc3ed7375583640445dbd";
  }

  static const char* value(const  ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbc4ecbd89e2dc3edULL;
  static const uint64_t static_value2 = 0x7375583640445dbdULL;
};

template<class ContainerAllocator>
struct DataType< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Modulo_Comunicaciones/msg_modo";
  }

  static const char* value(const  ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 modo\n\
";
  }

  static const char* value(const  ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.modo);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_modo_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Modulo_Comunicaciones::msg_modo_<ContainerAllocator> & v) 
  {
    s << indent << "modo: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.modo);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MODULO_COMUNICACIONES_MESSAGE_MSG_MODO_H

