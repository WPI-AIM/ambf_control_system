// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for rbdl_server/RBDLJacobianRequest
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
#include "rbdl_server/RBDLJacobian.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLJacobianRequest_common : public MATLABROSMsgInterface<rbdl_server::RBDLJacobian::Request> {
  public:
    virtual ~rbdl_server_msg_RBDLJacobianRequest_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLJacobian::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLJacobian::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLJacobianRequest_common::copy_from_struct(rbdl_server::RBDLJacobian::Request* msg, const matlab::data::Struct& arr,
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
        //q
        const matlab::data::TypedArray<double> q_arr = arr["Q"];
        size_t nelem = q_arr.getNumberOfElements();
        	msg->q.resize(nelem);
        	std::copy(q_arr.begin(), q_arr.begin()+nelem, msg->q.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Q' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Q' is wrong type; expected a double.");
    }
    try {
        //point
        const matlab::data::StructArray point_arr = arr["Point"];
        static auto msgClassPtr_point = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
        msgClassPtr_point->copy_from_struct(&msg->point,point_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Point' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Point' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLJacobianRequest_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLJacobian::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ModelName","BodyName","Q","Point"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLJacobianRequest");
    // model_name
    auto currentElement_model_name = (msg + ctr)->model_name;
    outArray[ctr]["ModelName"] = factory.createCharArray(currentElement_model_name);
    // body_name
    auto currentElement_body_name = (msg + ctr)->body_name;
    outArray[ctr]["BodyName"] = factory.createCharArray(currentElement_body_name);
    // q
    auto currentElement_q = (msg + ctr)->q;
    outArray[ctr]["Q"] = factory.createArray<rbdl_server::RBDLJacobian::Request::_q_type::const_iterator, double>({currentElement_q.size(),1}, currentElement_q.begin(), currentElement_q.end());
    // point
    auto currentElement_point = (msg + ctr)->point;
    static auto msgClassPtr_point = loader->createInstance<MATLABROSMsgInterface<geometry_msgs::Point>>("geometry_msgs_msg_Point_common");
    outArray[ctr]["Point"] = msgClassPtr_point->get_arr(factory, &currentElement_point, loader);
    }
    return std::move(outArray);
  }
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLJacobianResponse_common : public MATLABROSMsgInterface<rbdl_server::RBDLJacobian::Response> {
  public:
    virtual ~rbdl_server_msg_RBDLJacobianResponse_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLJacobian::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLJacobian::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLJacobianResponse_common::copy_from_struct(rbdl_server::RBDLJacobian::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //jacobian
        const matlab::data::StructArray jacobian_arr = arr["Jacobian"];
        static auto msgClassPtr_jacobian = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
        msgClassPtr_jacobian->copy_from_struct(&msg->jacobian,jacobian_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Jacobian' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Jacobian' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLJacobianResponse_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLJacobian::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Jacobian"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLJacobianResponse");
    // jacobian
    auto currentElement_jacobian = (msg + ctr)->jacobian;
    static auto msgClassPtr_jacobian = loader->createInstance<MATLABROSMsgInterface<std_msgs::Float64MultiArray>>("std_msgs_msg_Float64MultiArray_common");
    outArray[ctr]["Jacobian"] = msgClassPtr_jacobian->get_arr(factory, &currentElement_jacobian, loader);
    }
    return std::move(outArray);
  } 
class RBDL_SERVER_EXPORT rbdl_server_RBDLJacobian_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rbdl_server_RBDLJacobian_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rbdl_server_RBDLJacobian_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLJacobian::Request,rbdl_server_msg_RBDLJacobianRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLJacobian::Response,rbdl_server_msg_RBDLJacobianResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rbdl_server_RBDLJacobian_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLJacobian::Request,rbdl_server::RBDLJacobian::Request::ConstPtr,rbdl_server_msg_RBDLJacobianRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLJacobian::Response,rbdl_server::RBDLJacobian::Response::ConstPtr,rbdl_server_msg_RBDLJacobianResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rbdl_server_RBDLJacobian_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rbdl_server::RBDLJacobian::Request,rbdl_server::RBDLJacobian::Response,rbdl_server_msg_RBDLJacobianRequest_common,rbdl_server_msg_RBDLJacobianResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rbdl_server_RBDLJacobian_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rbdl_server::RBDLJacobian,rbdl_server::RBDLJacobian::Request,rbdl_server::RBDLJacobian::Response,rbdl_server_msg_RBDLJacobianRequest_common,rbdl_server_msg_RBDLJacobianResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLJacobianRequest_common, MATLABROSMsgInterface<rbdl_server::RBDLJacobian::Request>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLJacobianResponse_common, MATLABROSMsgInterface<rbdl_server::RBDLJacobian::Response>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_RBDLJacobian_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
