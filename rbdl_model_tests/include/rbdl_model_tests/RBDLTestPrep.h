#ifndef RBDLTESTPREP_H
#define RBDLTESTPREP_H

#include <atomic>
#include <iostream>
#include <future>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>

#include "rbdl_model_tests/rbdl_tests.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Constraints.h"

#include "rbdl_model/BuildRBDLModel.h"
#include "ambf_client/ambf_client.h"


//------------------------------------------------------------------------------
typedef BuildRBDLModel* BuildRBDLModelPtr;
typedef Client* AMBFClientPtr;
//------------------------------------------------------------------------------

const double TEST_PREC = 1.0e-2;



constexpr auto tenMill= 10000000;
class RBDLTestPrep
{
public:
    static RBDLTestPrep* getInstance(){
    RBDLTestPrep* sin= instance_.load(std::memory_order_acquire);
    if ( !sin )
    {
        std::lock_guard<std::mutex> myLock(myMutex_);
        sin = instance_.load(std::memory_order_relaxed);
        if( !sin )
        {
            sin = new RBDLTestPrep();
            instance_.store(sin,std::memory_order_release);
        }
    }
        return sin;
    }

    std::chrono::duration<double> getTime();
    AMBFClientPtr getAMBFClientInstance();
    BuildRBDLModelPtr getRBDLModelInstance();
private:
    RBDLTestPrep()= default;
    ~RBDLTestPrep()= default;
    RBDLTestPrep(const RBDLTestPrep&)= delete;
    RBDLTestPrep& operator=(const RBDLTestPrep&)= delete;

    static std::atomic<RBDLTestPrep*> instance_;
    static std::mutex myMutex_;

    const std::string actuator_config_file_ = "/mnt/OneTB/localcodebase/ambf_repos/ambf_models_with_inertia/KUKA/kuka.yaml";

};



#endif // RBDLTESTPREP_H
