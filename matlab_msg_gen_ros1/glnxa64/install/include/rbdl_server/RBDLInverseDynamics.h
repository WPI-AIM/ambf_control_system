// Generated by gencpp from file rbdl_server/RBDLInverseDynamics.msg
// DO NOT EDIT!


#ifndef RBDL_SERVER_MESSAGE_RBDLINVERSEDYNAMICS_H
#define RBDL_SERVER_MESSAGE_RBDLINVERSEDYNAMICS_H

#include <ros/service_traits.h>


#include <rbdl_server/RBDLInverseDynamicsRequest.h>
#include <rbdl_server/RBDLInverseDynamicsResponse.h>


namespace rbdl_server
{

struct RBDLInverseDynamics
{

typedef RBDLInverseDynamicsRequest Request;
typedef RBDLInverseDynamicsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RBDLInverseDynamics
} // namespace rbdl_server


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rbdl_server::RBDLInverseDynamics > {
  static const char* value()
  {
    return "c47e261a7e9e07c45f98872f45d9d184";
  }

  static const char* value(const ::rbdl_server::RBDLInverseDynamics&) { return value(); }
};

template<>
struct DataType< ::rbdl_server::RBDLInverseDynamics > {
  static const char* value()
  {
    return "rbdl_server/RBDLInverseDynamics";
  }

  static const char* value(const ::rbdl_server::RBDLInverseDynamics&) { return value(); }
};


// service_traits::MD5Sum< ::rbdl_server::RBDLInverseDynamicsRequest> should match 
// service_traits::MD5Sum< ::rbdl_server::RBDLInverseDynamics > 
template<>
struct MD5Sum< ::rbdl_server::RBDLInverseDynamicsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rbdl_server::RBDLInverseDynamics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseDynamicsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbdl_server::RBDLInverseDynamicsRequest> should match 
// service_traits::DataType< ::rbdl_server::RBDLInverseDynamics > 
template<>
struct DataType< ::rbdl_server::RBDLInverseDynamicsRequest>
{
  static const char* value()
  {
    return DataType< ::rbdl_server::RBDLInverseDynamics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseDynamicsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rbdl_server::RBDLInverseDynamicsResponse> should match 
// service_traits::MD5Sum< ::rbdl_server::RBDLInverseDynamics > 
template<>
struct MD5Sum< ::rbdl_server::RBDLInverseDynamicsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rbdl_server::RBDLInverseDynamics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseDynamicsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rbdl_server::RBDLInverseDynamicsResponse> should match 
// service_traits::DataType< ::rbdl_server::RBDLInverseDynamics > 
template<>
struct DataType< ::rbdl_server::RBDLInverseDynamicsResponse>
{
  static const char* value()
  {
    return DataType< ::rbdl_server::RBDLInverseDynamics >::value();
  }
  static const char* value(const ::rbdl_server::RBDLInverseDynamicsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RBDL_SERVER_MESSAGE_RBDLINVERSEDYNAMICS_H
