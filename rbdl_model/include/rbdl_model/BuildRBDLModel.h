#ifndef BUILDRBDLMODEL_H
#define BUILDRBDLMODEL_H
#include <iostream>
#include <unordered_map>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <vector>
#include <limits>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>
#include "application/Utilities.h"
#include "rbdl_model/ParseADF.h"

using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;
//------------------------------------------------------------------------------
typedef RigidBodyDynamics::Body rbdlBody;
typedef RigidBodyDynamics::Joint rbdlJoint;
typedef RigidBodyDynamics::JointType rbdlJointType;
typedef ParseADF* ParseADFPtr;
typedef Model* RBDLModelPtr;
//------------------------------------------------------------------------------
const double TEST_LAX {1.0e-7};
//------------------------------------------------------------------------------

class BuildRBDLModel
{
public:
  BuildRBDLModel(const std::string actuator_config_file, AMBFWrapperPtr ambfWrapperPtr);

  void PrintBody();
  void PrintJoint();
  void CleanUp();

  ~BuildRBDLModel(void);

  std::unordered_map<std::string, bodyParamPtr> inline GetRBDLBodyToObjectMap() { return bodyParamObjectMap_; }

  std::unordered_map<std::string, unsigned int> inline GetRBDLBodyToIDMap() { return rbdlObjectMap_; }
  std::unordered_map<std::string, unsigned int> inline GetRBDLJointToIDMap() { return joint_map; }

  // inline Model* RBDLModel() { return RBDLmodel_; }

  std::string inline BaseRigidBodyName() { return baseRigidBodyName_; }

  inline RBDLModelPtr RBDLModel() { return rbdlModelPtr_; }
  std::vector<std::string> GetAllBodyNames();
  unsigned int GetBodyId(const std::string bodyName);

  boost::optional<rbdlBody> GetRBDLBody(const std::string bodyName);
  std::unordered_map<std::string, jointParamPtr> GetJointChildren(std::string parent);
  std::vector<std::string> GetAllJointNames();

private:
  // bool ConnectToAMBF();
  
  // void RegisterRigidBodysPose();
  // void RegisterHomePoseTransformation();
  // void SetAMBFParams();

  bool BuildModel();

private:
  ParseADFPtr parseAdf_{nullptr};
  AMBFWrapperPtr ambfWrapperPtr_{nullptr};
  std::string baseRigidBodyName_{""};
  rigidBodyPtr baselinkHandler_{nullptr};
  std::vector<std::string> controlableJoints_;
  std::vector<std::string> endEffectorNodesName_;
  std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;

  // //                 <parent,                       <jointname, jointParamPtr>>
  // std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>> jointParamObjectMap_;
                    // <jointname, jointParamPtr>>
  std::unordered_map<std::string, jointParamPtr> jointParamObjectMap_;
  std::unordered_map<std::string, jointParamPtr>::iterator jointParamObjectMapItr_;

  std::unordered_map<std::string, unsigned int> rbdlObjectMap_;
  std::unordered_map<std::string, unsigned int> joint_map;
  std::unordered_map<std::string, unsigned int> ::iterator rbdl_object_map_itr_;


  std::vector<std::vector<std::string>> paths_;

  RBDLModelPtr rbdlModelPtr_{nullptr};
  VectorNd Q_;
  VectorNd QDot_;
  VectorNd QDDot_;
  VectorNd Tau_;
  std::map< std::string, unsigned int > rbdlmBodyMap_;
  std::map<std::string, unsigned int>::iterator rbdlmBodyMapItr_;
};

#endif // BUILDRBDLMODEL_H
