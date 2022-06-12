#include "rbdl_model/AMBFParams.h"
#include "application/Utilities.h"

AMBFParams::AMBFParams(const std::string name, rigidBodyPtr handler) :
  parentBodyName_(name), rigidBodyHandler_(handler) {}

void AMBFParams::ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig)
{
  controllableJointConfigs_ = jointConfig;
}

const Quaternion AMBFParams::GetQuternion()
{
  return Utilities::TFtoRBDLQuaternion(quat_w_n_tf_);
}

const Matrix3d AMBFParams::RotationMatrix()
{
  // Quaternion quat_w_n = EigenUtilities::TFtoRBDLQuaternion(quat_w_n_tf_);
	// return quat_w_n.toMatrix();
  Eigen::Quaterniond quat_w_n;
  quat_w_n.x() = quat_w_n_tf_.x();
  quat_w_n.y() = quat_w_n_tf_.y();
  quat_w_n.z() = quat_w_n_tf_.z();
  quat_w_n.w() = quat_w_n_tf_.w();

  return quat_w_n.normalized().toRotationMatrix();
}

const Vector3d AMBFParams::TranslationVector()
{
  return Utilities::TFtoRBDLVector(p_w_n_tf_);
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
