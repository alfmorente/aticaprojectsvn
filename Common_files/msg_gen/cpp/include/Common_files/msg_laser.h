/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Common_files/msg/msg_laser.msg */
#ifndef COMMON_FILES_MESSAGE_MSG_LASER_H
#define COMMON_FILES_MESSAGE_MSG_LASER_H
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


namespace Common_files
{
template <class ContainerAllocator>
struct msg_laser_ {
  typedef msg_laser_<ContainerAllocator> Type;

  msg_laser_()
  : id_laser(0)
  , angle()
  , distance()
  {
  }

  msg_laser_(const ContainerAllocator& _alloc)
  : id_laser(0)
  , angle(_alloc)
  , distance(_alloc)
  {
  }

  typedef uint8_t _id_laser_type;
  uint8_t id_laser;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _angle_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  angle;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _distance_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  distance;


  typedef boost::shared_ptr< ::Common_files::msg_laser_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Common_files::msg_laser_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_laser
typedef  ::Common_files::msg_laser_<std::allocator<void> > msg_laser;

typedef boost::shared_ptr< ::Common_files::msg_laser> msg_laserPtr;
typedef boost::shared_ptr< ::Common_files::msg_laser const> msg_laserConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Common_files::msg_laser_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Common_files::msg_laser_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Common_files

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Common_files::msg_laser_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Common_files::msg_laser_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Common_files::msg_laser_<ContainerAllocator> > {
  static const char* value() 
  {
    return "266af0804b5efb7b7f04ed8c5eec96b1";
  }

  static const char* value(const  ::Common_files::msg_laser_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x266af0804b5efb7bULL;
  static const uint64_t static_value2 = 0x7f04ed8c5eec96b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::Common_files::msg_laser_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Common_files/msg_laser";
  }

  static const char* value(const  ::Common_files::msg_laser_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Common_files::msg_laser_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 id_laser\n\
float32[] angle\n\
float32[] distance\n\
";
  }

  static const char* value(const  ::Common_files::msg_laser_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Common_files::msg_laser_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id_laser);
    stream.next(m.angle);
    stream.next(m.distance);
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
struct Printer< ::Common_files::msg_laser_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Common_files::msg_laser_<ContainerAllocator> & v) 
  {
    s << indent << "id_laser: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.id_laser);
    s << indent << "angle[]" << std::endl;
    for (size_t i = 0; i < v.angle.size(); ++i)
    {
      s << indent << "  angle[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.angle[i]);
    }
    s << indent << "distance[]" << std::endl;
    for (size_t i = 0; i < v.distance.size(); ++i)
    {
      s << indent << "  distance[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.distance[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // COMMON_FILES_MESSAGE_MSG_LASER_H

