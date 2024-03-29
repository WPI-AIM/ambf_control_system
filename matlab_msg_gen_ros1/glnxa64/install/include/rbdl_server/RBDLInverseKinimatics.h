// Generated by gencpp from file rbdl_server/RBDLInverseKinimatics.msg
// DO NOT EDIT!


#ifndef RBDL_SERVER_MESSAGE_RBDLINVERSEKINIMATICS_H
#define RBDL_SERVER_MESSAGE_RBDLINVERSEKINIMATICS_H

#include <ros/service_traits.h>


#include <rbdl_server/RBDLInverseKinimaticsRequest.h>
#include <rbdl_server/RBDLInverseKinimaticsResponse.h>


namespace rbdl_server
{

struct RBDLInverseKinimatics
{

typedef RBDLInverseKinimaticsRequest Request;
typedef RBDLInverseKinimaticsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RBDLInverseKinimatics
} // namespace rbdl_server


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rbdl_server::RBDLInverseKinimatics > {
  static const char* value()
  {
    return "7e93687695af62eb84a3727945f21e64";
  }

  static const char* value(const ::rbdl_server::RBDLInverseKinimatics&) { return value(); }
};

template<>
struct DataType< ::rbdl_server::RBDLInverseKinimatics > {
  static const char* value()
  {
    return "rbdl_server/RBDLInverseKinimatics";
  }

  static const char* value(const ::rbdl_server::RBDLInverseKinimatics&) { return value(); }
};


// service_traits::MD5Sum< ::rbdl_server::RBDLInverseKinimaticsRequest> should match 
// service_traits::MD5Sum< ::rbdl_server::RBDLInverseKinimatics > 
template<>
struct MD5Sum< ::rbdl_server::RBDLInverseKinimaticsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rbdl_server::RBDLInverseKinimatics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbdl_server::RBDLInverseKinimaticsRequest> should match 
// service_traits::DataType< ::rbdl_server::RBDLInverseKinimatics > 
template<>
struct DataType< ::rbdl_server::RBDLInverseKinimaticsRequest>
{
  static const char* value()
  {
    return DataType< ::rbdl_server::RBDLInverseKinimatics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rbdl_server::RBDLInverseKinimaticsResponse> should match 
// service_traits::MD5Sum< ::rbdl_server::RBDLInverseKinimatics > 
template<>
struct MD5Sum< ::rbdl_server::RBDLInverseKinimaticsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rbdl_server::RBDLInverseKinimatics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbdl_server::RBDLInverseKinimaticsResponse> should match 
// service_traits::DataType< ::rbdl_server::RBDLInverseKinimatics > 
template<>
struct DataType< ::rbdl_server::RBDLInverseKinimaticsResponse>
{
  static const char* value()
  {
    return DataType< ::rbdl_server::RBDLInverseKinimatics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseKinimaticsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RBDL_SERVER_MESSAGE_RBDLINVERSEKINIMATICS_H
