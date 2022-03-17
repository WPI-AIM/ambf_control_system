#include "rbdl_model_tests/AMBFParams.h"

AMBFParams::AMBFParams(const std::string name, rigidBodyPtr handler, const std::vector<std::string>& children, 
  std::vector<ControllableJointConfig> jConfigs, Matrix3d r, Vector3d p) :
  parentBodyName_(name), rigidBodyHandler_(handler), childrenJoints_(children), 
  controllableJointConfigs_(jConfigs), r_w_n_(r),
  p_w_n_(p) {}

void AMBFParams::ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig)
{
  controllableJointConfigs_ = jointConfig;
}

void AMBFParams::RotationMatrix(const Matrix3d r)
{
  r_w_n_ = r;
}

void AMBFParams::TranslationVector(const Vector3d p)
{
  p_w_n_ = p;
}