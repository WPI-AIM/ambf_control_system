#ifndef PSMBODYTESTS_H
#define PSMBODYTESTS_H

#include <UnitTest++.h>
#include <UnitTest++/UnitTest++.h>
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Body.h"
#include "rbdl_model/BuildRBDLModel.h"
#include "ambf_client/ambf_client.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


//------------------------------------------------------------------------------
typedef BuildRBDLModel* BuildRBDLModelPtr;
typedef Client* ClientPtr;
//------------------------------------------------------------------------------



const double TEST_PREC = 1.0e-7;

//class PSMBodyTests
//{
//public:
//    PSMBodyTests();
//    virtual ~PSMBodyTests(void);

//    virtual void init() = 0;
//    virtual void cleanUp();

//private:
//    const std::string actuator_config_file_ = "/localcodebase/ambfnags92/ambf/ambf_models/descriptions/multi-bodies/robots/blender-psm.yaml";
//    BuildRBDLModelPtr buildRBDLModelPtr_ = NULL;
//    std::unordered_map<std::string, bodyParamPtr> bodyParamObjectMap_; //getRBDLBodyToObjectMap()
//    std::unordered_map<std::string, std::unordered_map<std::string, jointParamPtr>> jointParamObjectMap_; //inline getRBDLJointToObjectMap() { return jointParamObjectMap_; }

//    std::unordered_map<std::string, unsigned int> rbdlObjectMap_; //inline getRBDLBodyToIDMap() { return rbdlObjectMap_; }


//    Client client_;
//    rigidBodyPtr psm_baselink_handler_;

//};

#endif // PSMBODYTESTS_H
