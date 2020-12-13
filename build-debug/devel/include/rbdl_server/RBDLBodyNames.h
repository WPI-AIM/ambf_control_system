// Generated by gencpp from file rbdl_server/RBDLBodyNames.msg
// DO NOT EDIT!


#ifndef RBDL_SERVER_MESSAGE_RBDLBODYNAMES_H
#define RBDL_SERVER_MESSAGE_RBDLBODYNAMES_H

#include <ros/service_traits.h>


#include <rbdl_server/RBDLBodyNamesRequest.h>
#include <rbdl_server/RBDLBodyNamesResponse.h>


namespace rbdl_server
{

struct RBDLBodyNames
{

typedef RBDLBodyNamesRequest Request;
typedef RBDLBodyNamesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RBDLBodyNames
} // namespace rbdl_server


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rbdl_server::RBDLBodyNames > {
  static const char* value()
  {
    return "dc7ae3609524b18034e49294a4ce670e";
  }

  static const char* value(const ::rbdl_server::RBDLBodyNames&) { return value(); }
};

template<>
struct DataType< ::rbdl_server::RBDLBodyNames > {
  static const char* value()
  {
    return "rbdl_server/RBDLBodyNames";
  }

  static const char* value(const ::rbdl_server::RBDLBodyNames&) { return value(); }
};


// service_traits::MD5Sum< ::rbdl_server::RBDLBodyNamesRequest> should match
// service_traits::MD5Sum< ::rbdl_server::RBDLBodyNames >
template<>
struct MD5Sum< ::rbdl_server::RBDLBodyNamesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rbdl_server::RBDLBodyNames >::value();
  }
  static const char* value(const ::rbdl_server::RBDLBodyNamesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbdl_server::RBDLBodyNamesRequest> should match
// service_traits::DataType< ::rbdl_server::RBDLBodyNames >
template<>
struct DataType< ::rbdl_server::RBDLBodyNamesRequest>
{
  static const char* value()
  {
    return DataType< ::rbdl_server::RBDLBodyNames >::value();
  }
  static const char* value(const ::rbdl_server::RBDLBodyNamesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rbdl_server::RBDLBodyNamesResponse> should match
// service_traits::MD5Sum< ::rbdl_server::RBDLBodyNames >
template<>
struct MD5Sum< ::rbdl_server::RBDLBodyNamesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rbdl_server::RBDLBodyNames >::value();
  }
  static const char* value(const ::rbdl_server::RBDLBodyNamesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbdl_server::RBDLBodyNamesResponse> should match
// service_traits::DataType< ::rbdl_server::RBDLBodyNames >
template<>
struct DataType< ::rbdl_server::RBDLBodyNamesResponse>
{
  static const char* value()
  {
    return DataType< ::rbdl_server::RBDLBodyNames >::value();
  }
  static const char* value(const ::rbdl_server::RBDLBodyNamesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RBDL_SERVER_MESSAGE_RBDLBODYNAMES_H