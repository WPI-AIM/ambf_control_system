#ifndef RBDLTESTPREP_H
#define RBDLTESTPREP_H
#include <UnitTest++.h>
#include <UnitTest++/UnitTest++.h>
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Body.h"
#include "rbdl_model/BuildRBDLModel.h"
//#include "ambf_client/ambf_client.h"


#include <atomic>
#include <iostream>
#include <future>
#include <mutex>
#include <thread>
#include <chrono>


#include <UnitTest++.h>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"
#include <vector>


using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


//------------------------------------------------------------------------------
typedef BuildRBDLModel* BuildRBDLModelPtr;
//typedef Client* ClientPtr;
//------------------------------------------------------------------------------

const double TEST_PREC = 1.0e-7;



constexpr auto tenMill= 10000000;
class RBDLTestPrep
{
public:
    static RBDLTestPrep* getInstance(){
    RBDLTestPrep* sin= instance.load(std::memory_order_acquire);
    if ( !sin ){
        std::lock_guard<std::mutex> myLock(myMutex);
        sin= instance.load(std::memory_order_relaxed);
        if( !sin ){
            sin= new RBDLTestPrep();
            instance.store(sin,std::memory_order_release);
        }
    }
    // volatile int dummy{};
    return sin;
    }

    std::chrono::duration<double> getTime();
//    ClientPtr getAMBFClientInstance();
    BuildRBDLModelPtr getRBDLModelInstance();
private:
    RBDLTestPrep()= default;
    ~RBDLTestPrep()= default;
    RBDLTestPrep(const RBDLTestPrep&)= delete;
    RBDLTestPrep& operator=(const RBDLTestPrep&)= delete;

    static std::atomic<RBDLTestPrep*> instance;
    static std::mutex myMutex;

//    const std::string actuator_config_file_ = "/localcodebase/ambf_repos/nag92/ambf/ambf_models/descriptions/multi-bodies/robots/blender-ecm.yaml";
    const std::string actuator_config_file_ = "/home/shreyas/Downloads/exohuman/exohuman_v4/exohuman_v4.yaml";

};



#endif // RBDLTESTPREP_H
