#include "rbdl_model/JointParam.h"

JointParam::JointParam(YAML::Node jointNode)
{
    // Declare all the yaml parameters that we want to look for
    Utilities utilities;

    YAML::Node name = jointNode["name"];
    if(!name.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", name in Joint Params");
    name_ = utilities.trimTrailingSpaces(name);

    YAML::Node parent = jointNode["parent"];
    if(!parent.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", parent name in Joint Params");
    parent_ = utilities.trimTrailingSpaces(parent);
    utilities.eraseSubStr(parent_, "BODY");

    YAML::Node child = jointNode["child"];
    if(!child.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", child name in Joint Params");
    child_ = utilities.trimTrailingSpaces(child);
    utilities.eraseSubStr(child_, "BODY");

    YAML::Node parent_pivot = jointNode["parent pivot"];
    if(!parent_pivot.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", parent pivot in Joint Params");
    parent_pivot_ = utilities.toXYZ(&parent_pivot);

    YAML::Node parent_axis = jointNode["parent axis"];
    if(!parent_axis.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", parent axis in Joint Params");
    parent_axis_ = utilities.toXYZ(&parent_axis);

    YAML::Node child_pivot = jointNode["child pivot"];
    if(!child_pivot.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", child pivot in Joint Params");
    child_pivot_ = utilities.toXYZ(&child_pivot);

    YAML::Node child_axis = jointNode["child axis"];
    if(!child_axis.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", child axis in Joint Params");
    child_axis_ = utilities.toXYZ(&child_axis);

    YAML::Node type = jointNode["type"];
    if(!type.IsDefined()) utilities.throwExceptionMessage("joint name: " + name_ + ", type in Joint Params");
    type_ = utilities.trimTrailingSpaces(type);
}

JointParam::JointParam(std::string name, std::string parent_name, std::string child, Vector3d parent_axis,
                       Vector3d parent_pivot, Vector3d child_axis, Vector3d child_pivot, std::string type) {
    name_ = (std::string(name)).c_str();
    parent_ = (std::string(parent_name)).c_str();
    child_ = (std::string(child)).c_str();
    parent_axis_ = parent_axis;
    parent_pivot_ = parent_pivot;
    child_axis_ = child_axis;
    child_pivot_ = child_pivot;
    type_ = (std::string(type)).c_str();
}

JointParam::~JointParam(void) {

}
