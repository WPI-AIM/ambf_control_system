// Copyright 2019-2020 The MathWorks, Inc.
// Common copy functions for controller_modules/JointControlRequest
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
#include "controller_modules/JointControl.h"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class CONTROLLER_MODULES_EXPORT controller_modules_msg_JointControlRequest_common : public MATLABROSMsgInterface<controller_modules::JointControl::Request> {
  public:
    virtual ~controller_modules_msg_JointControlRequest_common(){}
    virtual void copy_from_struct(controller_modules::JointControl::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const controller_modules::JointControl::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void controller_modules_msg_JointControlRequest_common::copy_from_struct(controller_modules::JointControl::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["Header"];
        static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Header' is wrong type; expected a struct.");
    }
    try {
        //joint_names
        const matlab::data::CellArray joint_names_cellarr = arr["JointNames"];
        size_t nelem = joint_names_cellarr.getNumberOfElements();
        for (size_t idx=0; idx < nelem; ++idx){
        	const matlab::data::CharArray joint_names_arr = joint_names_cellarr[idx];
        	msg->joint_names.push_back(joint_names_arr.toAscii());
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'JointNames' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'JointNames' is wrong type; expected a string.");
    }
    try {
        //controller_name
        const matlab::data::CharArray controller_name_arr = arr["ControllerName"];
        msg->controller_name = controller_name_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ControllerName' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ControllerName' is wrong type; expected a string.");
    }
    try {
        //desired
        const matlab::data::StructArray desired_arr = arr["Desired"];
        static auto msgClassPtr_desired = loader->createInstance<MATLABROSMsgInterface<trajectory_msgs::JointTrajectoryPoint>>("trajectory_msgs_msg_JointTrajectoryPoint_common");
        msgClassPtr_desired->copy_from_struct(&msg->desired,desired_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Desired' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Desired' is wrong type; expected a struct.");
    }
    try {
        //actual
        const matlab::data::StructArray actual_arr = arr["Actual"];
        static auto msgClassPtr_actual = loader->createInstance<MATLABROSMsgInterface<trajectory_msgs::JointTrajectoryPoint>>("trajectory_msgs_msg_JointTrajectoryPoint_common");
        msgClassPtr_actual->copy_from_struct(&msg->actual,actual_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Actual' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Actual' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T controller_modules_msg_JointControlRequest_common::get_arr(MDFactory_T& factory, const controller_modules::JointControl::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","JointNames","ControllerName","Desired","Actual"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("controller_modules/JointControlRequest");
    // header
    auto currentElement_header = (msg + ctr)->header;
    static auto msgClassPtr_header = loader->createInstance<MATLABROSMsgInterface<std_msgs::Header>>("std_msgs_msg_Header_common");
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // joint_names
    auto currentElement_joint_names = (msg + ctr)->joint_names;
    auto joint_namesoutCell = factory.createCellArray({currentElement_joint_names.size(),1});
    for(size_t idxin = 0; idxin < currentElement_joint_names.size(); ++ idxin){
    	joint_namesoutCell[idxin] = factory.createCharArray(currentElement_joint_names[idxin]);
    }
    outArray[ctr]["JointNames"] = joint_namesoutCell;
    // controller_name
    auto currentElement_controller_name = (msg + ctr)->controller_name;
    outArray[ctr]["ControllerName"] = factory.createCharArray(currentElement_controller_name);
    // desired
    auto currentElement_desired = (msg + ctr)->desired;
    static auto msgClassPtr_desired = loader->createInstance<MATLABROSMsgInterface<trajectory_msgs::JointTrajectoryPoint>>("trajectory_msgs_msg_JointTrajectoryPoint_common");
    outArray[ctr]["Desired"] = msgClassPtr_desired->get_arr(factory, &currentElement_desired, loader);
    // actual
    auto currentElement_actual = (msg + ctr)->actual;
    static auto msgClassPtr_actual = loader->createInstance<MATLABROSMsgInterface<trajectory_msgs::JointTrajectoryPoint>>("trajectory_msgs_msg_JointTrajectoryPoint_common");
    outArray[ctr]["Actual"] = msgClassPtr_actual->get_arr(factory, &currentElement_actual, loader);
    }
    return std::move(outArray);
  }
class CONTROLLER_MODULES_EXPORT controller_modules_msg_JointControlResponse_common : public MATLABROSMsgInterface<controller_modules::JointControl::Response> {
  public:
    virtual ~controller_modules_msg_JointControlResponse_common(){}
    virtual void copy_from_struct(controller_modules::JointControl::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const controller_modules::JointControl::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void controller_modules_msg_JointControlResponse_common::copy_from_struct(controller_modules::JointControl::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //control_output
        const matlab::data::StructArray control_output_arr = arr["ControlOutput"];
        static auto msgClassPtr_control_output = loader->createInstance<MATLABROSMsgInterface<trajectory_msgs::JointTrajectoryPoint>>("trajectory_msgs_msg_JointTrajectoryPoint_common");
        msgClassPtr_control_output->copy_from_struct(&msg->control_output,control_output_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'ControlOutput' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'ControlOutput' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T controller_modules_msg_JointControlResponse_common::get_arr(MDFactory_T& factory, const controller_modules::JointControl::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","ControlOutput"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("controller_modules/JointControlResponse");
    // control_output
    auto currentElement_control_output = (msg + ctr)->control_output;
    static auto msgClassPtr_control_output = loader->createInstance<MATLABROSMsgInterface<trajectory_msgs::JointTrajectoryPoint>>("trajectory_msgs_msg_JointTrajectoryPoint_common");
    outArray[ctr]["ControlOutput"] = msgClassPtr_control_output->get_arr(factory, &currentElement_control_output, loader);
    }
    return std::move(outArray);
  } 
class CONTROLLER_MODULES_EXPORT controller_modules_JointControl_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~controller_modules_JointControl_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          controller_modules_JointControl_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<controller_modules::JointControl::Request,controller_modules_msg_JointControlRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<controller_modules::JointControl::Response,controller_modules_msg_JointControlResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          controller_modules_JointControl_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<controller_modules::JointControl::Request,controller_modules::JointControl::Request::ConstPtr,controller_modules_msg_JointControlRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<controller_modules::JointControl::Response,controller_modules::JointControl::Response::ConstPtr,controller_modules_msg_JointControlResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          controller_modules_JointControl_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<controller_modules::JointControl::Request,controller_modules::JointControl::Response,controller_modules_msg_JointControlRequest_common,controller_modules_msg_JointControlResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          controller_modules_JointControl_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<controller_modules::JointControl,controller_modules::JointControl::Request,controller_modules::JointControl::Response,controller_modules_msg_JointControlRequest_common,controller_modules_msg_JointControlResponse_common>>();
  }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(controller_modules_msg_JointControlRequest_common, MATLABROSMsgInterface<controller_modules::JointControl::Request>)
CLASS_LOADER_REGISTER_CLASS(controller_modules_msg_JointControlResponse_common, MATLABROSMsgInterface<controller_modules::JointControl::Response>)
CLASS_LOADER_REGISTER_CLASS(controller_modules_JointControl_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
