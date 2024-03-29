// Generated by gencpp from file rbdl_server/RBDLModelAlignmentRequest.msg
// DO NOT EDIT!


#ifndef RBDL_SERVER_MESSAGE_RBDLMODELALIGNMENTREQUEST_H
#define RBDL_SERVER_MESSAGE_RBDLMODELALIGNMENTREQUEST_H


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
struct RBDLModelAlignmentRequest_
{
  typedef RBDLModelAlignmentRequest_<ContainerAllocator> Type;

  RBDLModelAlignmentRequest_()
    : model_name()
    , names()  {
    }
  RBDLModelAlignmentRequest_(const ContainerAllocator& _alloc)
    : model_name(_alloc)
    , names(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _model_name_type;
  _model_name_type model_name;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _names_type;
  _names_type names;





  typedef boost::shared_ptr< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RBDLModelAlignmentRequest_

typedef ::rbdl_server::RBDLModelAlignmentRequest_<std::allocator<void> > RBDLModelAlignmentRequest;

typedef boost::shared_ptr< ::rbdl_server::RBDLModelAlignmentRequest > RBDLModelAlignmentRequestPtr;
typedef boost::shared_ptr< ::rbdl_server::RBDLModelAlignmentRequest const> RBDLModelAlignmentRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "07e770c1dc5b4e3e58279541dc6391a0";
  }

  static const char* value(const ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x07e770c1dc5b4e3eULL;
  static const uint64_t static_value2 = 0x58279541dc6391a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rbdl_server/RBDLModelAlignmentRequest";
  }

  static const char* value(const ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string model_name\n"
"string[] names\n"
;
  }

  static const char* value(const ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.model_name);
      stream.next(m.names);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RBDLModelAlignmentRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rbdl_server::RBDLModelAlignmentRequest_<ContainerAllocator>& v)
  {
    s << indent << "model_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.model_name);
    s << indent << "names[]" << std::endl;
    for (size_t i = 0; i < v.names.size(); ++i)
    {
      s << indent << "  names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.names[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RBDL_SERVER_MESSAGE_RBDLMODELALIGNMENTREQUEST_H
