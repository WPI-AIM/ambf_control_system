// Generated by gencpp from file controller_modules/ControllerList.msg
// DO NOT EDIT!


#ifndef CONTROLLER_MODULES_MESSAGE_CONTROLLERLIST_H
#define CONTROLLER_MODULES_MESSAGE_CONTROLLERLIST_H

#include <ros/service_traits.h>


#include <controller_modules/ControllerListRequest.h>
#include <controller_modules/ControllerListResponse.h>


namespace controller_modules
{

struct ControllerList
{

typedef ControllerListRequest Request;
typedef ControllerListResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ControllerList
} // namespace controller_modules


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::controller_modules::ControllerList > {
  static const char* value()
  {
    return "8242783a458107f102bce0800c333f0a";
  }

  static const char* value(const ::controller_modules::ControllerList&) { return value(); }
};

template<>
struct DataType< ::controller_modules::ControllerList > {
  static const char* value()
  {
    return "controller_modules/ControllerList";
  }

  static const char* value(const ::controller_modules::ControllerList&) { return value(); }
};


// service_traits::MD5Sum< ::controller_modules::ControllerListRequest> should match 
// service_traits::MD5Sum< ::controller_modules::ControllerList > 
template<>
struct MD5Sum< ::controller_modules::ControllerListRequest>
{
  static const char* value()
  {
    return MD5Sum< ::controller_modules::ControllerList >::value();
  }
  static const char* value(const ::controller_modules::ControllerListRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::controller_modules::ControllerListRequest> should match 
// service_traits::DataType< ::controller_modules::ControllerList > 
template<>
struct DataType< ::controller_modules::ControllerListRequest>
{
  static const char* value()
  {
    return DataType< ::controller_modules::ControllerList >::value();
  }
  static const char* value(const ::controller_modules::ControllerListRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::controller_modules::ControllerListResponse> should match 
// service_traits::MD5Sum< ::controller_modules::ControllerList > 
template<>
struct MD5Sum< ::controller_modules::ControllerListResponse>
{
  static const char* value()
  {
    return MD5Sum< ::controller_modules::ControllerList >::value();
  }
  static const char* value(const ::controller_modules::ControllerListResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::controller_modules::ControllerListResponse> should match 
// service_traits::DataType< ::controller_modules::ControllerList > 
template<>
struct DataType< ::controller_modules::ControllerListResponse>
{
  static const char* value()
  {
    return DataType< ::controller_modules::ControllerList >::value();
  }
  static const char* value(const ::controller_modules::ControllerListResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CONTROLLER_MODULES_MESSAGE_CONTROLLERLIST_H
