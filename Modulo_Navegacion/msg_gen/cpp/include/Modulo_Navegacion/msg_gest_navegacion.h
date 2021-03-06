/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/Navegacion_Atica/Modulo_Navegacion/msg/msg_gest_navegacion.msg */
#ifndef MODULO_NAVEGACION_MESSAGE_MSG_GEST_NAVEGACION_H
#define MODULO_NAVEGACION_MESSAGE_MSG_GEST_NAVEGACION_H
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

#include "tf/tfMessage.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"

namespace Modulo_Navegacion
{
template <class ContainerAllocator>
struct msg_gest_navegacion_ {
  typedef msg_gest_navegacion_<ContainerAllocator> Type;

  msg_gest_navegacion_()
  : mensajeTF()
  , odometria()
  , laserInfo()
  , posicion()
  {
  }

  msg_gest_navegacion_(const ContainerAllocator& _alloc)
  : mensajeTF(_alloc)
  , odometria(_alloc)
  , laserInfo(_alloc)
  , posicion(_alloc)
  {
  }

  typedef  ::tf::tfMessage_<ContainerAllocator>  _mensajeTF_type;
   ::tf::tfMessage_<ContainerAllocator>  mensajeTF;

  typedef  ::nav_msgs::Odometry_<ContainerAllocator>  _odometria_type;
   ::nav_msgs::Odometry_<ContainerAllocator>  odometria;

  typedef  ::sensor_msgs::LaserScan_<ContainerAllocator>  _laserInfo_type;
   ::sensor_msgs::LaserScan_<ContainerAllocator>  laserInfo;

  typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _posicion_type;
   ::geometry_msgs::PoseStamped_<ContainerAllocator>  posicion;


  typedef boost::shared_ptr< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct msg_gest_navegacion
typedef  ::Modulo_Navegacion::msg_gest_navegacion_<std::allocator<void> > msg_gest_navegacion;

typedef boost::shared_ptr< ::Modulo_Navegacion::msg_gest_navegacion> msg_gest_navegacionPtr;
typedef boost::shared_ptr< ::Modulo_Navegacion::msg_gest_navegacion const> msg_gest_navegacionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace Modulo_Navegacion

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e80f53167c8cde7cbde4a732ad762d62";
  }

  static const char* value(const  ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe80f53167c8cde7cULL;
  static const uint64_t static_value2 = 0xbde4a732ad762d62ULL;
};

template<class ContainerAllocator>
struct DataType< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Modulo_Navegacion/msg_gest_navegacion";
  }

  static const char* value(const  ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> > {
  static const char* value() 
  {
    return "tf/tfMessage mensajeTF\n\
nav_msgs/Odometry odometria\n\
sensor_msgs/LaserScan laserInfo\n\
geometry_msgs/PoseStamped posicion\n\
================================================================================\n\
MSG: tf/tfMessage\n\
geometry_msgs/TransformStamped[] transforms\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TransformStamped\n\
# This expresses a transform from coordinate frame header.frame_id\n\
# to the coordinate frame child_frame_id\n\
#\n\
# This message is mostly used by the \n\
# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. \n\
# See its documentation for more information.\n\
\n\
Header header\n\
string child_frame_id # the frame id of the child frame\n\
Transform transform\n\
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
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: nav_msgs/Odometry\n\
# This represents an estimate of a position and velocity in free space.  \n\
# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n\
# The twist in this message should be specified in the coordinate frame given by the child_frame_id\n\
Header header\n\
string child_frame_id\n\
geometry_msgs/PoseWithCovariance pose\n\
geometry_msgs/TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertainty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: sensor_msgs/LaserScan\n\
# Single scan from a planar laser range-finder\n\
#\n\
# If you have another ranging device with different behavior (e.g. a sonar\n\
# array), please find or create a different message, since applications\n\
# will make fairly laser-specific assumptions about this data\n\
\n\
Header header            # timestamp in the header is the acquisition time of \n\
                         # the first ray in the scan.\n\
                         #\n\
                         # in frame frame_id, angles are measured around \n\
                         # the positive Z axis (counterclockwise, if Z is up)\n\
                         # with zero angle being forward along the x axis\n\
                         \n\
float32 angle_min        # start angle of the scan [rad]\n\
float32 angle_max        # end angle of the scan [rad]\n\
float32 angle_increment  # angular distance between measurements [rad]\n\
\n\
float32 time_increment   # time between measurements [seconds] - if your scanner\n\
                         # is moving, this will be used in interpolating position\n\
                         # of 3d points\n\
float32 scan_time        # time between scans [seconds]\n\
\n\
float32 range_min        # minimum range value [m]\n\
float32 range_max        # maximum range value [m]\n\
\n\
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)\n\
float32[] intensities    # intensity data [device-specific units].  If your\n\
                         # device does not provide intensities, please leave\n\
                         # the array empty.\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
";
  }

  static const char* value(const  ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.mensajeTF);
    stream.next(m.odometria);
    stream.next(m.laserInfo);
    stream.next(m.posicion);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct msg_gest_navegacion_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::Modulo_Navegacion::msg_gest_navegacion_<ContainerAllocator> & v) 
  {
    s << indent << "mensajeTF: ";
s << std::endl;
    Printer< ::tf::tfMessage_<ContainerAllocator> >::stream(s, indent + "  ", v.mensajeTF);
    s << indent << "odometria: ";
s << std::endl;
    Printer< ::nav_msgs::Odometry_<ContainerAllocator> >::stream(s, indent + "  ", v.odometria);
    s << indent << "laserInfo: ";
s << std::endl;
    Printer< ::sensor_msgs::LaserScan_<ContainerAllocator> >::stream(s, indent + "  ", v.laserInfo);
    s << indent << "posicion: ";
s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.posicion);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MODULO_NAVEGACION_MESSAGE_MSG_GEST_NAVEGACION_H

