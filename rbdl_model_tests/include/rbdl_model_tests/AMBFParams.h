#include <iostream>
#include "ambf_client/ambf_client.h"

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
    tf::Quaternion quat_tf, tf::Vector3 p_tf);
  
  inline const std::string ParentBodyName() { return parentBodyName; }
  inline rigidBodyPtr RididBodyHandler() { return rigidBodyHandler; }
  inline const std::vector<std::string> ChildrenJoints() { return childrenJoints; }
  inline const std::vector<ControllableJointConfig> ControllableJointConfigs() { return controllableJointConfigs; }
  inline const tf::Quaternion RotationQuaternionTF() { return quat_w_n_tf; }
  inline const tf::Vector3 TranslationVectorTF() { return p_w_n_tf; }

  void ControllableJointConfigs(const std::vector<ControllableJointConfig>& jointConfig);
  void RotationQuaternionTF(const tf::Quaternion quat_tf);
  void TranslationVectorTF(const tf::Vector3 p_tf);
  ~AMBFParams();

private:
  const std::string parentBodyName;
  rigidBodyPtr rigidBodyHandler = nullptr;
  const std::vector<std::string> childrenJoints;
  std::vector<ControllableJointConfig> controllableJointConfigs;
  tf::Quaternion quat_w_n_tf;
  tf::Vector3 p_w_n_tf;
};