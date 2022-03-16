#include "rbdl_model_tests/AMBFParams.h"

AMBFParams::AMBFParams(const std::string name, 
  rigidBodyPtr handler, const std::vector<std::string>& children, 
  std::vector<ControllableJointConfig> jConfigs, tf::Quaternion quat_tf, 
  tf::Vector3 p_tf) :
  parentBodyName(name), rigidBodyHandler(handler), childrenJoints(children), 
  controllableJointConfigs(jConfigs), quat_w_n_tf(quat_tf),
  p_w_n_tf(p_tf) {}

void AMBFParams::ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig)
{
  controllableJointConfigs = jointConfig;
}

void AMBFParams::RotationQuaternionTF(const tf::Quaternion quat_tf)
{
  quat_w_n_tf = quat_tf;
}

void AMBFParams::TranslationVectorTF(const tf::Vector3 p_tf)
{
  p_w_n_tf = p_tf;
}