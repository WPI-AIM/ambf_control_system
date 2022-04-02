#include <iostream>
#include "ambf_client/ambf_client.h"
#include "rbdl/rbdl_math.h"
#include <tf/LinearMath/Transform.h>
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


typedef struct ControllableJointConfig
{
  ControllableJointConfig() : jointName(), jointLowerLimit(), jointUpperLimit() {}
  ControllableJointConfig(std::string name, float lowerLimit, float upperLimit)
    : jointName(name), jointLowerLimit(lowerLimit), jointUpperLimit(upperLimit) {}

  std::string jointName;
  float jointLowerLimit;
  float jointUpperLimit;
} ControllableJointConfig;

class AMBFParams
{
public:
  AMBFParams(const std::string name, rigidBodyPtr handler);
  
  inline const std::string ParentBodyName() { return parentBodyName_; }
  inline rigidBodyPtr RididBodyHandler() { return rigidBodyHandler_; }
  inline const std::vector<ControllableJointConfig> ControllableJointConfigs() { return controllableJointConfigs_; }
  const Matrix3d RotationMatrix();
  const Vector3d TranslationVector();
  inline const float QDesired() { return qDesired_; }

  void ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig);
  void QuaternionTF(const tf::Quaternion q);
  void TranslationVectorTF(const tf::Vector3 p);
  void QDesired(float q);
  ~AMBFParams();

private:
  const std::string parentBodyName_;
  rigidBodyPtr rigidBodyHandler_ = nullptr;
  const std::vector<std::string> childrenJoints_;
  std::vector<ControllableJointConfig> controllableJointConfigs_;
  // Matrix3d r_w_n_;
  // Vector3d p_w_n_;
  tf::Quaternion quat_w_n_tf_;
  tf::Vector3 p_w_n_tf_;

  float qDesired_{0.0};
};