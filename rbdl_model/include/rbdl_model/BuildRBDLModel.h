#ifndef PARSE_YAML_H
#define PARSE_YAML_H
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <unordered_map>
#include "rbdl_model/BodyParam.h"
#include "rbdl_model/JointParam.h"
#include "rbdl_model/Utilities.h"
#include <queue>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_math.h>
#include <sstream>
#include <unordered_set>
#include <vector>
#include <boost/optional.hpp>
#include <rbdl_model/RBDLModelErrors.h>
#include <limits>

using namespace RigidBodyDynamics;
using namespace Math;
using namespace RigidBodyDynamics::Math;
//------------------------------------------------------------------------------
typedef BodyParam* bodyParamPtr;
typedef JointParam* jointParamPtr;
typedef RigidBodyDynamics::Body rbdlBody;
typedef RigidBodyDynamics::Joint rbdlJoint;
typedef RigidBodyDynamics::JointType rbdlJointType;
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
    Model inline getRBDLModel() { return *RBDLmodel_; }
    std::string inline getBaseRigidBody() { return baseRigidBody_; }

    std::vector<std::string> getAllBodyNames();
    unsigned int getBodyId(const std::string bodyName);

    boost::optional<rbdlBody> getRBDLBody(const std::string bodyName);
    std::unordered_map<std::string, jointParamPtr> getJointChildren(std::string parent);

private:
    void getNamespace();
    bool getBodies();
    bool getJoints();
    bool findBaseNode();
    void addDummyBaseJoint();

    bool buildBodyTree();
    bool buildModel();

    std::string blender_namespace_;
    YAML::Node baseNode_;
    std::string actuator_config_file_;
    std::string baseRigidBody_;

    const std::string base_parent_name_ = "world";
    std::string base_joint_name_;
    Model *RBDLmodel_ = NULL;

    std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_;

    //                 <parent,                       <jointname, jointParamPtr>>
    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>> jointParamObjectMap_;

    std::unordered_map<std::string, unsigned int> rbdlObjectMap_;
    std::unordered_map<std::string, unsigned int> ::iterator rbdl_object_map_itr_;


    const rbdlJointType getRBDLJointType(std::string joint_type);
    unsigned int addBodyToRBDL(std::string parent_name, unsigned int parent_id, std::string joint_name, std::string child_name);

    // Below Maps are used for Getters only. They dont play a role in model creation.
//    std::unordered_map<std::string, rbdlBody> rbdlBodyMap_;
    std::unordered_map<std::string, boost::optional<rbdlBody>> rbdlBodyMap_;

};

#endif // PARSE_YAML_H
