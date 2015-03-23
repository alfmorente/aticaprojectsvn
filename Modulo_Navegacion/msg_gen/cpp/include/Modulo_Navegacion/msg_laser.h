/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Navegacion_Atica/Modulo_Navegacion/msg/msg_laser.msg */
#ifndef MODULO_NAVEGACION_MESSAGE_MSG_LASER_H
#define MODULO_NAVEGACION_MESSAGE_MSG_LASER_H
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


namespace Modulo_Navegacion
{
template <class ContainerAllocator>
struct msg_laser_ {
  typedef msg_laser_<ContainerAllocator> Type;

  msg_laser_()
  : angulos()
  , distancias()
  {
  }

  msg_laser_(const ContainerAllocator& _alloc)
  : angulos(_alloc)
  , distancias(_alloc)
  {
  }

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _angulos_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  angulos;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _distancias_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  distancias;


  typedef boost::shared_ptr< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Modulo_Navegacion::msg_laser_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_laser
typedef  ::Modulo_Navegacion::msg_laser_<std::allocator<void> > msg_laser;

typedef boost::shared_ptr< ::Modulo_Navegacion::msg_laser> msg_laserPtr;
typedef boost::shared_ptr< ::Modulo_Navegacion::msg_laser const> msg_laserConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Modulo_Navegacion::msg_laser_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Modulo_Navegacion

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Modulo_Navegacion::msg_laser_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2583b36e4ed5b0750e4c27e6e92888c7";
  }

  static const char* value(const  ::Modulo_Navegacion::msg_laser_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2583b36e4ed5b075ULL;
  static const uint64_t static_value2 = 0x0e4c27e6e92888c7ULL;
};

template<class ContainerAllocator>
struct DataType< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Modulo_Navegacion/msg_laser";
  }

  static const char* value(const  ::Modulo_Navegacion::msg_laser_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32[] angulos\n\
float32[] distancias\n\
";
  }

  static const char* value(const  ::Modulo_Navegacion::msg_laser_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.angulos);
    stream.next(m.distancias);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_laser_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Modulo_Navegacion::msg_laser_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Modulo_Navegacion::msg_laser_<ContainerAllocator> & v) 
  {
    s << indent << "angulos[]" << std::endl;
    for (size_t i = 0; i < v.angulos.size(); ++i)
    {
      s << indent << "  angulos[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.angulos[i]);
    }
    s << indent << "distancias[]" << std::endl;
    for (size_t i = 0; i < v.distancias.size(); ++i)
    {
      s << indent << "  distancias[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.distancias[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // MODULO_NAVEGACION_MESSAGE_MSG_LASER_H

