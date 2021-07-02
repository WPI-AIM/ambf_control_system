// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for rbdl_server/RBDLBodyNamesRequest
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
#include "rbdl_server/RBDLBodyNames.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLBodyNamesRequest_common : public MATLABROSMsgInterface<rbdl_server::RBDLBodyNames::Request> {
  public:
    virtual ~rbdl_server_msg_RBDLBodyNamesRequest_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLBodyNames::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLBodyNames::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLBodyNamesRequest_common::copy_from_struct(rbdl_server::RBDLBodyNames::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //model_name
        const matlab::data::CharArray model_name_arr = arr["ModelName"];
        msg->model_name = model_name_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ModelName' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ModelName' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLBodyNamesRequest_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLBodyNames::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ModelName"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLBodyNamesRequest");
    // model_name
    auto currentElement_model_name = (msg + ctr)->model_name;
    outArray[ctr]["ModelName"] = factory.createCharArray(currentElement_model_name);
    }
    return std::move(outArray);
  }
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLBodyNamesResponse_common : public MATLABROSMsgInterface<rbdl_server::RBDLBodyNames::Response> {
  public:
    virtual ~rbdl_server_msg_RBDLBodyNamesResponse_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLBodyNames::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLBodyNames::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLBodyNamesResponse_common::copy_from_struct(rbdl_server::RBDLBodyNames::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //names
        const matlab::data::CellArray names_cellarr = arr["Names"];
        size_t nelem = names_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray names_arr = names_cellarr[idx];
        	msg->names.push_back(names_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Names' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Names' is wrong type; expected a string.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLBodyNamesResponse_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLBodyNames::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Names"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLBodyNamesResponse");
    // names
    auto currentElement_names = (msg + ctr)->names;
    auto namesoutCell = factory.createCellArray({currentElement_names.size(),1});
    for(size_t idxin = 0; idxin < currentElement_names.size(); ++ idxin){
    	namesoutCell[idxin] = factory.createCharArray(currentElement_names[idxin]);
    }
    outArray[ctr]["Names"] = namesoutCell;
    }
    return std::move(outArray);
  } 
class RBDL_SERVER_EXPORT rbdl_server_RBDLBodyNames_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rbdl_server_RBDLBodyNames_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rbdl_server_RBDLBodyNames_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLBodyNames::Request,rbdl_server_msg_RBDLBodyNamesRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLBodyNames::Response,rbdl_server_msg_RBDLBodyNamesResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rbdl_server_RBDLBodyNames_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLBodyNames::Request,rbdl_server::RBDLBodyNames::Request::ConstPtr,rbdl_server_msg_RBDLBodyNamesRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLBodyNames::Response,rbdl_server::RBDLBodyNames::Response::ConstPtr,rbdl_server_msg_RBDLBodyNamesResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rbdl_server_RBDLBodyNames_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rbdl_server::RBDLBodyNames::Request,rbdl_server::RBDLBodyNames::Response,rbdl_server_msg_RBDLBodyNamesRequest_common,rbdl_server_msg_RBDLBodyNamesResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rbdl_server_RBDLBodyNames_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rbdl_server::RBDLBodyNames,rbdl_server::RBDLBodyNames::Request,rbdl_server::RBDLBodyNames::Response,rbdl_server_msg_RBDLBodyNamesRequest_common,rbdl_server_msg_RBDLBodyNamesResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLBodyNamesRequest_common, MATLABROSMsgInterface<rbdl_server::RBDLBodyNames::Request>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLBodyNamesResponse_common, MATLABROSMsgInterface<rbdl_server::RBDLBodyNames::Response>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_RBDLBodyNames_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
