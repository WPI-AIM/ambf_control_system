// Generated by gencpp from file rbdl_server/RBDLInverseKinimaticsResponse.msg
// DO NOT EDIT!


#ifndef RBDL_SERVER_MESSAGE_RBDLINVERSEKINIMATICSRESPONSE_H
#define RBDL_SERVER_MESSAGE_RBDLINVERSEKINIMATICSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rbdl_server
{
template <class ContainerAllocator>
struct RBDLInverseKinimaticsResponse_
{
  typedef RBDLInverseKinimaticsResponse_<ContainerAllocator> Type;

  RBDLInverseKinimaticsResponse_()
    : q_res()
    , worked(false)  {
    }
  RBDLInverseKinimaticsResponse_(const ContainerAllocator& _alloc)
    : q_res(_alloc)
    , worked(false)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _q_res_type;
  _q_res_type q_res;

   typedef uint8_t _worked_type;
  _worked_type worked;





  typedef boost::shared_ptr< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RBDLInverseKinimaticsResponse_

typedef ::rbdl_server::RBDLInverseKinimaticsResponse_<std::allocator<void> > RBDLInverseKinimaticsResponse;

typedef boost::shared_ptr< ::rbdl_server::RBDLInverseKinimaticsResponse > RBDLInverseKinimaticsResponsePtr;
typedef boost::shared_ptr< ::rbdl_server::RBDLInverseKinimaticsResponse const> RBDLInverseKinimaticsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rbdl_server

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f3258c0e1786aef3f710d2b34dce99b7";
  }

  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf3258c0e1786aef3ULL;
  static const uint64_t static_value2 = 0xf710d2b34dce99b7ULL;
};

template<class ContainerAllocator>
struct DataType< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbdl_server/RBDLInverseKinimaticsResponse";
  }

  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] q_res\n"
"bool worked\n"
"\n"
;
  }

  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.q_res);
      stream.next(m.worked);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RBDLInverseKinimaticsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbdl_server::RBDLInverseKinimaticsResponse_<ContainerAllocator>& v)
  {
    s << indent << "q_res[]" << std::endl;
    for (size_t i = 0; i < v.q_res.size(); ++i)
    {
      s << indent << "  q_res[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.q_res[i]);
    }
    s << indent << "worked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.worked);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBDL_SERVER_MESSAGE_RBDLINVERSEKINIMATICSRESPONSE_H
