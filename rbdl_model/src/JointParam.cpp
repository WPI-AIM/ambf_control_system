#include "rbdl_model/JointParam.h"

JointParam::JointParam(YAML::Node jointNode)
{
  // Declare all the yaml parameters that we want to look for
  // Utilities utilities;

  YAML::Node name = jointNode["name"];
  if(!name.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", name in Joint Params");
  name_ = Utilities::TrimTrailingSpaces(name);

  YAML::Node parent = jointNode["parent"];
  if(!parent.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", parent name in Joint Params");
  parent_ = Utilities::TrimTrailingSpaces(parent);
  Utilities::EraseSubStr(parent_, "BODY");

  YAML::Node child = jointNode["child"];
  if(!child.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", child name in Joint Params");
  child_ = Utilities::TrimTrailingSpaces(child);
  Utilities::EraseSubStr(child_, "BODY");

  YAML::Node parent_pivot = jointNode["parent pivot"];
  if(!parent_pivot.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", parent pivot in Joint Params");
  parent_pivot_ = Utilities::ToXYZ(&parent_pivot);

  YAML::Node parent_axis = jointNode["parent axis"];
  if(!parent_axis.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", parent axis in Joint Params");
  parent_axis_ = Utilities::ToXYZ(&parent_axis);

  YAML::Node child_pivot = jointNode["child pivot"];
  if(!child_pivot.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", child pivot in Joint Params");
  child_pivot_ = Utilities::ToXYZ(&child_pivot);

  YAML::Node child_axis = jointNode["child axis"];
  if(!child_axis.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", child axis in Joint Params");
  child_axis_ = Utilities::ToXYZ(&child_axis);

  YAML::Node joint_limits = jointNode["joint limits"];
  if(!joint_limits.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", joint limits in Joint Params");

  YAML::Node joint_limits_high = joint_limits["high"];
  if(!joint_limits_high.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", joint limit high in Joint Params");
  joint_limits_high_ = Utilities::ToDouble(joint_limits_high);

  YAML::Node joint_limits_low = joint_limits["low"];
  if(!joint_limits_low.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", joint limit low in Joint Params");
  joint_limits_low_ = Utilities::ToDouble(joint_limits_low);

  YAML::Node type = jointNode["type"];
  if(!type.IsDefined()) Utilities::ThrowMissingFieldException("joint name: " + name_ + ", type in Joint Params");
  type_ = Utilities::TrimTrailingSpaces(type);
  // As of now only prismatic and revolute joints are supported
  if(!(type_.compare("revolute") == 0 || type_.compare("prismatic") == 0)) 
    Utilities::ThrowUnsupportedJointException(name_, type_);

  YAML::Node offset = jointNode["offset"];
  if(!offset.IsDefined()) offset_ = 0.0;
  else offset_ = Utilities::ToDouble(offset);

  // weight added to select the desired joint path using Dijkstra's Algorithm
  // If not weight found in YAML its set to 1. Dijkstra cannot have negative weights
  YAML::Node weight = jointNode["weight"];
  if(!weight.IsDefined()) weight_ = 1;
  else weight_ = Utilities::ToInt(weight);
  if(weight_ < 1) Utilities::ThrowInvalidValueException("joint name: " + name_ + ", weight should be at least 1");

  // ToDo: Make sure that only supported joints are included in the model
  // std::string joint_type = type.as<std::string>();
  // if(!(joint_type.compare("revolute") || !(joint_type.compare("prismatic"))
  //     utilities.throwExceptionMessage("joint name: " + name_ + ", child axis in Joint Params");
}

JointParam::JointParam(std::string name, std::string parent_name, std::string child, Vector3d parent_axis,
  Vector3d parent_pivot, Vector3d child_axis, Vector3d child_pivot, std::string type) 
{
  name_ = (std::string(name)).c_str();
  parent_ = (std::string(parent_name)).c_str();
  child_ = (std::string(child)).c_str();
  parent_axis_ = parent_axis;
  parent_pivot_ = parent_pivot;
  child_axis_ = child_axis;
  child_pivot_ = child_pivot;
  type_ = (std::string(type)).c_str();
  // add weight if needed. This constructor may not be needed
}

JointParam::~JointParam(void) {

}