/* Auto-generated by genmsg_cpp for file /home/atica/catkin_ws/src/CITIUS/CITIUS_Control_Communication/srv/srv_focusDirect.srv */
#ifndef CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_FOCUSDIRECT_H
#define CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_FOCUSDIRECT_H
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

#include "ros/service_traits.h"




namespace CITIUS_Control_Communication
{
template <class ContainerAllocator>
struct srv_focusDirectRequest_ {
  typedef srv_focusDirectRequest_<ContainerAllocator> Type;

  srv_focusDirectRequest_()
  : focusDirect(0)
  {
  }

  srv_focusDirectRequest_(const ContainerAllocator& _alloc)
  : focusDirect(0)
  {
  }

  typedef uint8_t _focusDirect_type;
  uint8_t focusDirect;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct srv_focusDirectRequest
typedef  ::CITIUS_Control_Communication::srv_focusDirectRequest_<std::allocator<void> > srv_focusDirectRequest;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectRequest> srv_focusDirectRequestPtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectRequest const> srv_focusDirectRequestConstPtr;



template <class ContainerAllocator>
struct srv_focusDirectResponse_ {
  typedef srv_focusDirectResponse_<ContainerAllocator> Type;

  srv_focusDirectResponse_()
  : ret(false)
  {
  }

  srv_focusDirectResponse_(const ContainerAllocator& _alloc)
  : ret(false)
  {
  }

  typedef uint8_t _ret_type;
  uint8_t ret;


  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct srv_focusDirectResponse
typedef  ::CITIUS_Control_Communication::srv_focusDirectResponse_<std::allocator<void> > srv_focusDirectResponse;

typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectResponse> srv_focusDirectResponsePtr;
typedef boost::shared_ptr< ::CITIUS_Control_Communication::srv_focusDirectResponse const> srv_focusDirectResponseConstPtr;


struct srv_focusDirect
{

typedef srv_focusDirectRequest Request;
typedef srv_focusDirectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct srv_focusDirect
} // namespace CITIUS_Control_Communication

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2a4faf04ae6f6f9415d5149d5b46cf30";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2a4faf04ae6f6f94ULL;
  static const uint64_t static_value2 = 0x15d5149d5b46cf30ULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_focusDirectRequest";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 focusDirect\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e2cc9e9d8c464550830df49c160979ad";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe2cc9e9d8c464550ULL;
  static const uint64_t static_value2 = 0x830df49c160979adULL;
};

template<class ContainerAllocator>
struct DataType< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_focusDirectResponse";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool ret\n\
\n\
\n\
";
  }

  static const char* value(const  ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.focusDirect);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct srv_focusDirectRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ret);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct srv_focusDirectResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<CITIUS_Control_Communication::srv_focusDirect> {
  static const char* value() 
  {
    return "a4dfa2720320ab393b086bef03523e56";
  }

  static const char* value(const CITIUS_Control_Communication::srv_focusDirect&) { return value(); } 
};

template<>
struct DataType<CITIUS_Control_Communication::srv_focusDirect> {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_focusDirect";
  }

  static const char* value(const CITIUS_Control_Communication::srv_focusDirect&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a4dfa2720320ab393b086bef03523e56";
  }

  static const char* value(const CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_focusDirect";
  }

  static const char* value(const CITIUS_Control_Communication::srv_focusDirectRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a4dfa2720320ab393b086bef03523e56";
  }

  static const char* value(const CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CITIUS_Control_Communication/srv_focusDirect";
  }

  static const char* value(const CITIUS_Control_Communication::srv_focusDirectResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // CITIUS_CONTROL_COMMUNICATION_SERVICE_SRV_FOCUSDIRECT_H
