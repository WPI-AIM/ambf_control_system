#include "rbdl_model/BodyParam.h"

BodyParam::BodyParam(YAML::Node bodyNode)
{
    // Declare all the yaml parameters that we want to look for
    YAML::Node name = bodyNode["name"];
    if(!name.IsDefined()) RBDLUtilities::ThrowMissingFieldException("name in Body Params");
    name_ = RBDLUtilities::TrimTrailingSpaces(name);

    YAML::Node mass = bodyNode["mass"];
    if(!mass.IsDefined()) RBDLUtilities::ThrowMissingFieldException("mass in Body Params");
    mass_ = mass.as<double>();
    if(mass_ == 0.0) mass_ = 0.001;

    YAML::Node inertia = bodyNode["inertia"];
    if(!inertia.IsDefined()) RBDLUtilities::ThrowMissingFieldException("inertia in Body Params");
    inertia_ = RBDLUtilities::VectorToMatrix3d(&inertia);

    //-------------------//
    YAML::Node location = bodyNode["location"];
    if(!location.IsDefined()) RBDLUtilities::ThrowMissingFieldException("location in Body Params");

    YAML::Node location_position = location["position"];
    if(!location_position.IsDefined()) RBDLUtilities::ThrowMissingFieldException("location position in Body Params");
    location_position_ = RBDLUtilities::ToXYZ(&location_position);

    YAML::Node location_orientation = location["orientation"];
    if(!location_orientation.IsDefined()) RBDLUtilities::ThrowMissingFieldException("location orientation in Body Params");
    location_orientation_ = RBDLUtilities::ToRPY(&location_orientation);
    //-------------------//

    //-------------------//
    YAML::Node inertial_offset = bodyNode["inertial offset"];
    if(!inertial_offset.IsDefined()) RBDLUtilities::ThrowMissingFieldException("inertia offset in Body Params");

    YAML::Node inertial_offset_position = inertial_offset["position"];
    if(!inertial_offset_position.IsDefined()) RBDLUtilities::ThrowMissingFieldException("inertia offset position in Body Params");
    inertial_offset_position_ = RBDLUtilities::ToXYZ(&inertial_offset_position);

    YAML::Node inertial_offset_orientation = inertial_offset["orientation"];
    if(!inertial_offset_orientation.IsDefined()) RBDLUtilities::ThrowMissingFieldException("inertia offset orientation in Body Params");
    inertial_offset_orientation_ = RBDLUtilities::ToRPY(&inertial_offset_orientation);
    //-------------------//

    YAML::Node passive = bodyNode["passive"];
    if(!passive.IsDefined()) RBDLUtilities::ThrowMissingFieldException("passive in Body Params");
    passive_ = RBDLUtilities::ToBool(passive);
}

BodyParam::~BodyParam() {

}
