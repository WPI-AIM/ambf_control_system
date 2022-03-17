#include <iostream>
#include "ambf_client/ambf_client.h"
#include "rbdl/rbdl_math.h"
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
  AMBFParams(const std::string name, rigidBodyPtr handler, 
    const std::vector<std::string>& children, std::vector<ControllableJointConfig> jConfig,
    Matrix3d r, Vector3d p);
  
  inline const std::string ParentBodyName() { return parentBodyName_; }
  inline rigidBodyPtr RididBodyHandler() { return rigidBodyHandler_; }
  inline const std::vector<std::string> ChildrenJoints() { return childrenJoints_; }
  inline const std::vector<ControllableJointConfig> ControllableJointConfigs() { return controllableJointConfigs_; }
  inline const Matrix3d RotationMatrix() { return r_w_n_; }
  inline const Vector3d TranslationVector() { return p_w_n_; }

  void ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig);
  void RotationMatrix(const Matrix3d r);
  void TranslationVector(const Vector3d p);
  ~AMBFParams();

private:
  const std::string parentBodyName_;
  rigidBodyPtr rigidBodyHandler_ = nullptr;
  const std::vector<std::string> childrenJoints_;
  std::vector<ControllableJointConfig> controllableJointConfigs_;
  Matrix3d r_w_n_;
  Vector3d p_w_n_;
};