#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <vector>
#include <boost/optional.hpp>
#include <boost/bimap.hpp>
#include <limits>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>

#include "rbdl_model/BodyParam.h"
#include "rbdl_model/JointParam.h"
#include "rbdl_model/ModelGraph.h"

// #include "ambf_client/ambf_client.h"
#include "application/Prep.h"
#include "application/Utilities.h"

using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;
//------------------------------------------------------------------------------
typedef BodyParam* bodyParamPtr;
typedef JointParam* jointParamPtr;
typedef RigidBodyDynamics::Body rbdlBody;
typedef RigidBodyDynamics::Joint rbdlJoint;
typedef RigidBodyDynamics::JointType rbdlJointType;
typedef boost::bimap<std::string, int> bmStrInt;
typedef bmStrInt::left_map::const_iterator bmLeftConstItr;
typedef bmStrInt::right_map::const_iterator bmRightConstItr;
//------------------------------------------------------------------------------

class BuildRBDLModel
{
public:
  BuildRBDLModel(const std::string actuator_config_file);

  void printBody();
  void printJoint();
  void cleanUp();

  ~BuildRBDLModel(void);

  std::unordered_map<std::string, bodyParamPtr> inline getRBDLBodyToObjectMap() { return bodyParamObjectMap_; }

  std::unordered_map<std::string, unsigned int> inline getRBDLBodyToIDMap() { return rbdlObjectMap_; }
  std::unordered_map<std::string, unsigned int> inline getRBDLJointToIDMap() { return joint_map; }

  inline Model* getRBDLModel() { return RBDLmodel_; }

  std::string inline getBaseRigidBody() { return baseRigidBody_; }

  std::vector<std::string> getAllBodyNames();
  unsigned int getBodyId(const std::string bodyName);

  boost::optional<rbdlBody> getRBDLBody(const std::string bodyName);
  std::unordered_map<std::string, jointParamPtr> getJointChildren(std::string parent);
  std::vector<std::string> getAllJointNames();

private:
  bool ConnectToAMBF();
  void getNamespace();
  
  template< class MapType >
  void PrintMap(const MapType & map, const std::string & separator, std::ostream & os);
  
  template< class MapType, typename T, typename R>
  R GetValueFromMap(const MapType & map, T t, R r);

  bool getBodies();
  bool getJoints();
  bool buildModelSequence();
  bool buildModel();

private:
  AMBFClientPtr ambfClientPtr_{nullptr};
  std::string blender_namespace_;
  YAML::Node baseNode_;
  std::string actuator_config_file_;
  std::string baseRigidBody_;

  const std::string base_parent_name_ = "world";
  std::string base_joint_name_;
  Model *RBDLmodel_ = NULL;
  std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;

  // //                 <parent,                       <jointname, jointParamPtr>>
  // std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>> jointParamObjectMap_;
                    // <jointname, jointParamPtr>>
  std::unordered_map<std::string, jointParamPtr> jointParamObjectMap_;
  std::unordered_map<std::string, jointParamPtr>::iterator jointParamObjectMapItr_;

  std::unordered_map<std::string, unsigned int> rbdlObjectMap_;
  std::unordered_map<std::string, unsigned int> joint_map;
  std::unordered_map<std::string, unsigned int> ::iterator rbdl_object_map_itr_;


  // const rbdlJointType getRBDLJointType(std::string joint_type);
  // unsigned int addBodyToRBDL(std::string parent_name, unsigned int parent_id, std::string joint_name, std::string child_name);

  // Below Maps are used for Getters only. They dont play a role in model creation.

  bmStrInt bodyNameHash_;
  std::vector<std::vector<std::string>> paths_;
};

#endif // PARSE_YAML_H
