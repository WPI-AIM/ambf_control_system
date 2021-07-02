// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for rbdl_server/RBDLInverseDynamicsRequest
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
#include "rbdl_server/RBDLInverseDynamics.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLInverseDynamicsRequest_common : public MATLABROSMsgInterface<rbdl_server::RBDLInverseDynamics::Request> {
  public:
    virtual ~rbdl_server_msg_RBDLInverseDynamicsRequest_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLInverseDynamics::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseDynamics::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLInverseDynamicsRequest_common::copy_from_struct(rbdl_server::RBDLInverseDynamics::Request* msg, const matlab::data::Struct& arr,
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
        //qd
        const matlab::data::TypedArray<double> qd_arr = arr["Qd"];
        size_t nelem = qd_arr.getNumberOfElements();
        	msg->qd.resize(nelem);
        	std::copy(qd_arr.begin(), qd_arr.begin()+nelem, msg->qd.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Qd' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Qd' is wrong type; expected a double.");
    }
    try {
        //qdd
        const matlab::data::TypedArray<double> qdd_arr = arr["Qdd"];
        size_t nelem = qdd_arr.getNumberOfElements();
        	msg->qdd.resize(nelem);
        	std::copy(qdd_arr.begin(), qdd_arr.begin()+nelem, msg->qdd.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Qdd' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Qdd' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLInverseDynamicsRequest_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseDynamics::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ModelName","Q","Qd","Qdd"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLInverseDynamicsRequest");
    // model_name
    auto currentElement_model_name = (msg + ctr)->model_name;
    outArray[ctr]["ModelName"] = factory.createCharArray(currentElement_model_name);
    // q
    auto currentElement_q = (msg + ctr)->q;
    outArray[ctr]["Q"] = factory.createArray<rbdl_server::RBDLInverseDynamics::Request::_q_type::const_iterator, double>({currentElement_q.size(),1}, currentElement_q.begin(), currentElement_q.end());
    // qd
    auto currentElement_qd = (msg + ctr)->qd;
    outArray[ctr]["Qd"] = factory.createArray<rbdl_server::RBDLInverseDynamics::Request::_qd_type::const_iterator, double>({currentElement_qd.size(),1}, currentElement_qd.begin(), currentElement_qd.end());
    // qdd
    auto currentElement_qdd = (msg + ctr)->qdd;
    outArray[ctr]["Qdd"] = factory.createArray<rbdl_server::RBDLInverseDynamics::Request::_qdd_type::const_iterator, double>({currentElement_qdd.size(),1}, currentElement_qdd.begin(), currentElement_qdd.end());
    }
    return std::move(outArray);
  }
class RBDL_SERVER_EXPORT rbdl_server_msg_RBDLInverseDynamicsResponse_common : public MATLABROSMsgInterface<rbdl_server::RBDLInverseDynamics::Response> {
  public:
    virtual ~rbdl_server_msg_RBDLInverseDynamicsResponse_common(){}
    virtual void copy_from_struct(rbdl_server::RBDLInverseDynamics::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseDynamics::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void rbdl_server_msg_RBDLInverseDynamicsResponse_common::copy_from_struct(rbdl_server::RBDLInverseDynamics::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //tau
        const matlab::data::TypedArray<double> tau_arr = arr["Tau"];
        size_t nelem = tau_arr.getNumberOfElements();
        	msg->tau.resize(nelem);
        	std::copy(tau_arr.begin(), tau_arr.begin()+nelem, msg->tau.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Tau' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Tau' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T rbdl_server_msg_RBDLInverseDynamicsResponse_common::get_arr(MDFactory_T& factory, const rbdl_server::RBDLInverseDynamics::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Tau"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("rbdl_server/RBDLInverseDynamicsResponse");
    // tau
    auto currentElement_tau = (msg + ctr)->tau;
    outArray[ctr]["Tau"] = factory.createArray<rbdl_server::RBDLInverseDynamics::Response::_tau_type::const_iterator, double>({currentElement_tau.size(),1}, currentElement_tau.begin(), currentElement_tau.end());
    }
    return std::move(outArray);
  } 
class RBDL_SERVER_EXPORT rbdl_server_RBDLInverseDynamics_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~rbdl_server_RBDLInverseDynamics_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          rbdl_server_RBDLInverseDynamics_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLInverseDynamics::Request,rbdl_server_msg_RBDLInverseDynamicsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<rbdl_server::RBDLInverseDynamics::Response,rbdl_server_msg_RBDLInverseDynamicsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          rbdl_server_RBDLInverseDynamics_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLInverseDynamics::Request,rbdl_server::RBDLInverseDynamics::Request::ConstPtr,rbdl_server_msg_RBDLInverseDynamicsRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<rbdl_server::RBDLInverseDynamics::Response,rbdl_server::RBDLInverseDynamics::Response::ConstPtr,rbdl_server_msg_RBDLInverseDynamicsResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          rbdl_server_RBDLInverseDynamics_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<rbdl_server::RBDLInverseDynamics::Request,rbdl_server::RBDLInverseDynamics::Response,rbdl_server_msg_RBDLInverseDynamicsRequest_common,rbdl_server_msg_RBDLInverseDynamicsResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          rbdl_server_RBDLInverseDynamics_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<rbdl_server::RBDLInverseDynamics,rbdl_server::RBDLInverseDynamics::Request,rbdl_server::RBDLInverseDynamics::Response,rbdl_server_msg_RBDLInverseDynamicsRequest_common,rbdl_server_msg_RBDLInverseDynamicsResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLInverseDynamicsRequest_common, MATLABROSMsgInterface<rbdl_server::RBDLInverseDynamics::Request>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_msg_RBDLInverseDynamicsResponse_common, MATLABROSMsgInterface<rbdl_server::RBDLInverseDynamics::Response>)
CLASS_LOADER_REGISTER_CLASS(rbdl_server_RBDLInverseDynamics_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
