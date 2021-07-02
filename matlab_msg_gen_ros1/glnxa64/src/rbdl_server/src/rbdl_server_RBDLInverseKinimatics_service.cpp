// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for rbdl_server/RBDLInverseKinimaticsRequest
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
#include "rbdl_server/RBDLInverseKinimatics.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLInverseKinimaticsRequest_common : public MATLABROSMsgInterface<rbdl_server::RBDLInverseKinimatics::Request> {
  public:
    virtual ~rbdl_server_msg_RBDLInverseKinimaticsRequest_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLInverseKinimatics::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseKinimatics::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLInverseKinimaticsRequest_common::copy_from_struct(rbdl_server::RBDLInverseKinimatics::Request* msg, const matlab::data::Struct& arr,
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
    try {
        //body_name
        const matlab::data::CharArray body_name_arr = arr["BodyName"];
        msg->body_name = body_name_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BodyName' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BodyName' is wrong type; expected a string.");
    }
    try {
        //target
        const matlab::data::StructArray target_arr = arr["Target"];
        static auto msgClassPtr_target = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
        msgClassPtr_target->copy_from_struct(&msg->target,target_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Target' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Target' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLInverseKinimaticsRequest_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseKinimatics::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ModelName","BodyName","Target"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLInverseKinimaticsRequest");
    // model_name
    auto currentElement_model_name = (msg + ctr)->model_name;
    outArray[ctr]["ModelName"] = factory.createCharArray(currentElement_model_name);
    // body_name
    auto currentElement_body_name = (msg + ctr)->body_name;
    outArray[ctr]["BodyName"] = factory.createCharArray(currentElement_body_name);
    // target
    auto currentElement_target = (msg + ctr)->target;
    static auto msgClassPtr_target = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
    outArray[ctr]["Target"] = msgClassPtr_target->get_arr(factory, &currentElement_target, loader);
    }
    return std::move(outArray);
  }
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLInverseKinimaticsResponse_common : public MATLABROSMsgInterface<rbdl_server::RBDLInverseKinimatics::Response> {
  public:
    virtual ~rbdl_server_msg_RBDLInverseKinimaticsResponse_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLInverseKinimatics::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseKinimatics::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLInverseKinimaticsResponse_common::copy_from_struct(rbdl_server::RBDLInverseKinimatics::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //q_res
        const matlab::data::TypedArray<double> q_res_arr = arr["QRes"];
        size_t nelem = q_res_arr.getNumberOfElements();
        	msg->q_res.resize(nelem);
        	std::copy(q_res_arr.begin(), q_res_arr.begin()+nelem, msg->q_res.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'QRes' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'QRes' is wrong type; expected a double.");
    }
    try {
        //worked
        const matlab::data::TypedArray<bool> worked_arr = arr["Worked"];
        msg->worked = worked_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Worked' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Worked' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLInverseKinimaticsResponse_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseKinimatics::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","QRes","Worked"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLInverseKinimaticsResponse");
    // q_res
    auto currentElement_q_res = (msg + ctr)->q_res;
    outArray[ctr]["QRes"] = factory.createArray<rbdl_server::RBDLInverseKinimatics::Response::_q_res_type::const_iterator, double>({currentElement_q_res.size(),1}, currentElement_q_res.begin(), currentElement_q_res.end());
    // worked
    auto currentElement_worked = (msg + ctr)->worked;
    outArray[ctr]["Worked"] = factory.createScalar(static_cast<bool>(currentElement_worked));
    }
    return std::move(outArray);
  } 
class RBDL_SERVER_EXPORT rbdl_server_RBDLInverseKinimatics_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rbdl_server_RBDLInverseKinimatics_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rbdl_server_RBDLInverseKinimatics_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLInverseKinimatics::Request,rbdl_server_msg_RBDLInverseKinimaticsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLInverseKinimatics::Response,rbdl_server_msg_RBDLInverseKinimaticsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rbdl_server_RBDLInverseKinimatics_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLInverseKinimatics::Request,rbdl_server::RBDLInverseKinimatics::Request::ConstPtr,rbdl_server_msg_RBDLInverseKinimaticsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLInverseKinimatics::Response,rbdl_server::RBDLInverseKinimatics::Response::ConstPtr,rbdl_server_msg_RBDLInverseKinimaticsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rbdl_server_RBDLInverseKinimatics_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rbdl_server::RBDLInverseKinimatics::Request,rbdl_server::RBDLInverseKinimatics::Response,rbdl_server_msg_RBDLInverseKinimaticsRequest_common,rbdl_server_msg_RBDLInverseKinimaticsResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rbdl_server_RBDLInverseKinimatics_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rbdl_server::RBDLInverseKinimatics,rbdl_server::RBDLInverseKinimatics::Request,rbdl_server::RBDLInverseKinimatics::Response,rbdl_server_msg_RBDLInverseKinimaticsRequest_common,rbdl_server_msg_RBDLInverseKinimaticsResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLInverseKinimaticsRequest_common, MATLABROSMsgInterface<rbdl_server::RBDLInverseKinimatics::Request>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLInverseKinimaticsResponse_common, MATLABROSMsgInterface<rbdl_server::RBDLInverseKinimatics::Response>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_RBDLInverseKinimatics_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
