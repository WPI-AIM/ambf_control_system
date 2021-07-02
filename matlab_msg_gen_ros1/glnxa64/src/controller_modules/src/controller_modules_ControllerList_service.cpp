// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for controller_modules/ControllerListRequest
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "controller_modules/ControllerList.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class CONTROLLER_MODULES_EXPORT controller_modules_msg_ControllerListRequest_common : public MATLABROSMsgInterface<controller_modules::ControllerList::Request> {
  public:
    virtual ~controller_modules_msg_ControllerListRequest_common(){}
    virtual void copy_from_struct(controller_modules::ControllerList::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const controller_modules::ControllerList::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void controller_modules_msg_ControllerListRequest_common::copy_from_struct(controller_modules::ControllerList::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
  }
  //----------------------------------------------------------------------------
  MDArray_T controller_modules_msg_ControllerListRequest_common::get_arr(MDFactory_T& factory, const controller_modules::ControllerList::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("controller_modules/ControllerListRequest");
    }
    return std::move(outArray);
  }
class CONTROLLER_MODULES_EXPORT controller_modules_msg_ControllerListResponse_common : public MATLABROSMsgInterface<controller_modules::ControllerList::Response> {
  public:
    virtual ~controller_modules_msg_ControllerListResponse_common(){}
    virtual void copy_from_struct(controller_modules::ControllerList::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const controller_modules::ControllerList::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void controller_modules_msg_ControllerListResponse_common::copy_from_struct(controller_modules::ControllerList::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //controllers
        const matlab::data::CellArray controllers_cellarr = arr["Controllers"];
        size_t nelem = controllers_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray controllers_arr = controllers_cellarr[idx];
        	msg->controllers.push_back(controllers_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Controllers' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Controllers' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T controller_modules_msg_ControllerListResponse_common::get_arr(MDFactory_T& factory, const controller_modules::ControllerList::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Controllers"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("controller_modules/ControllerListResponse");
    // controllers
    auto currentElement_controllers = (msg + ctr)->controllers;
    auto controllersoutCell = factory.createCellArray({currentElement_controllers.size(),1});
    for(size_t idxin = 0; idxin < currentElement_controllers.size(); ++ idxin){
    	controllersoutCell[idxin] = factory.createCharArray(currentElement_controllers[idxin]);
    }
    outArray[ctr]["Controllers"] = controllersoutCell;
    }
    return std::move(outArray);
  } 
class CONTROLLER_MODULES_EXPORT controller_modules_ControllerList_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~controller_modules_ControllerList_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          controller_modules_ControllerList_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<controller_modules::ControllerList::Request,controller_modules_msg_ControllerListRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<controller_modules::ControllerList::Response,controller_modules_msg_ControllerListResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          controller_modules_ControllerList_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<controller_modules::ControllerList::Request,controller_modules::ControllerList::Request::ConstPtr,controller_modules_msg_ControllerListRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<controller_modules::ControllerList::Response,controller_modules::ControllerList::Response::ConstPtr,controller_modules_msg_ControllerListResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          controller_modules_ControllerList_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<controller_modules::ControllerList::Request,controller_modules::ControllerList::Response,controller_modules_msg_ControllerListRequest_common,controller_modules_msg_ControllerListResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          controller_modules_ControllerList_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<controller_modules::ControllerList,controller_modules::ControllerList::Request,controller_modules::ControllerList::Response,controller_modules_msg_ControllerListRequest_common,controller_modules_msg_ControllerListResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(controller_modules_msg_ControllerListRequest_common, MATLABROSMsgInterface<controller_modules::ControllerList::Request>)
CLASS_LOADER_REGISTER_CLASS(controller_modules_msg_ControllerListResponse_common, MATLABROSMsgInterface<controller_modules::ControllerList::Response>)
CLASS_LOADER_REGISTER_CLASS(controller_modules_ControllerList_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
