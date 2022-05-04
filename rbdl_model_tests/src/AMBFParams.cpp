#include "rbdl_model_tests/AMBFParams.h"
#include "rbdl_model_tests/EigenUtilities.h"

AMBFParams::AMBFParams(const std::string name, rigidBodyPtr handler) :
  parentBodyName_(name), rigidBodyHandler_(handler) {}

void AMBFParams::ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig)
{
  controllableJointConfigs_ = jointConfig;
}

const Matrix3d AMBFParams::RotationMatrix()
{
  Quaternion quat_w_n = EigenUtilities::TFtoRBDLQuaternion(quat_w_n_tf_);
	return quat_w_n.toMatrix();
}

const Vector3d AMBFParams::TranslationVector()
{
  return EigenUtilities::TFtoRBDLVector(p_w_n_tf_);
}

void AMBFParams::QuaternionTF(const tf::Quaternion q)
{
  quat_w_n_tf_ = q;
}

void AMBFParams::TranslationVectorTF(const tf::Vector3 p)
{
  p_w_n_tf_ = p;
}

void AMBFParams::QDesired(float q)
{
  qDesired_ = q;
}
